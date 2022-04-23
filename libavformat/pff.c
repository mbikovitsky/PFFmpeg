/*
 * Still Life 2 PFF demuxer
 * Copyright (c) 2022 Michael Bikovitsky
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <inttypes.h>

#include "libavutil/avstring.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/avassert.h"
#include "libavutil/log.h"
#include "libavutil/internal.h"
#include "libavutil/error.h"
#include "libavutil/macros.h"
#include "libavutil/buffer.h"

#include "libavcodec/packet.h"
#include "libavcodec/codec_id.h"
#include "libavcodec/codec_par.h"

#include "avformat.h"
#include "internal.h"
#include "avio.h"
#include "avlanguage.h"

#include "fifo_demuxer.h"


/*
 * See https://bikodbg.com/blog/2022/04/still-life/ for the full story
 * about this file format.
 *
 * The basics:
 *
 * - A PFF file consists of a header, which describes the video and audio streams
 *   in the file, followed by frames. This demuxer supports only VIDEO_DDS
 *   and SOUND_VORBIS streams (AFAIK there are the only formats used in any
 *   PFF files in existence).
 * - Each frame contains a timestamp and zero or more "sections".
 * - A video section for a VIDEO_DDS stream is a single compressed DDS image.
 * - A SOUND_VORBIS audio section is part of an Ogg stream that encapsulates
 *   a single Vorbis stream. If you stitch all audio sections of a single stream
 *   together you'll get a valid Ogg-Vorbis file.
 *
 * This demuxer works as follows:
 *
 * 1. In pff_read_header, we process the file header and set up the video AVStream.
 *    We also record the languages of all audio streams.
 * 2. Then, in each call to pff_read_packet, we advance to the next *section*, and:
 *    1. If it's a video section, we decompress (pff_process_video_section)
 *       and return the DDS image.
 *    2. If it's a sound section, we feed its contents to the OGG demuxer (oggdec.c)
 *       and, as long as the demuxer can give us frames, we return them in each call to
 *       pff_read_packet. After no more packets are available we continue as usual.
 *       See pff_process_sound_section and pff_try_read_audio_frame.
 *
 * The idea is to exhaust each section before moving on to the next. A video section
 * contains only one image, but a sound section can contain several audio frames.
 *
 * What we're doing here is, umm, interesting. The only API we have for pulling
 * frames out of a demuxer is av_read_frame, which eventually calls the demuxer's
 * read_packet funtion. If the demuxer hits an error in this function -- like, say,
 * EOF -- there is no guarantee that we can safely "restart" it in the future.
 *
 * But, we're doing exactly that! We're calling av_read_frame until we hit AVERROR_EOF,
 * then feeding some more data, and calling av_read_frame again. This is not a good
 * idea in the general case.
 *
 * The whole thing relies on two assumptions:
 *
 * 1. That the Ogg demuxer can handle a restart if it hits an EOF *at a page boundary*.
 *    That is, if we hit an EOF in the middle of a page, all bets are off. If not,
 *    it's fine.
 * 2. That the PFF format does not split Ogg pages across frames.
 *
 * Given assumption 2, we are guaranteed that we always have a whole number of pages
 * when calling av_read_frame. Given assumption 1, even if the pages we have don't
 * have a whole number of Ogg *packets* in them, the demuxer will handle it
 * (i.e. return the packets when it has enough data).
 *
 * Assumption 2 is safe. The files I have don't split Ogg pages, and AFAIK there
 * aren't any other in existence. Assumption 1 is... safe-ish? The basic packet
 * parsing code in oggdec.c has remained unchanged since at least 2007
 * (https://git.ffmpeg.org/gitweb/ffmpeg.git/commit/a0ddef24ced504abeaccb4d23e051831a0998298)
 * so it's probably fine.
 *
 * But if not, it's not like I'm submitting this to the actual FFmpeg codebase...
 *
 * Also, for the curious, we're not checking that there is at most one video section
 * in a frame. ¯\_(ツ)_/¯
 */


/**
 * The file magic.
 */
#define PFF_MAGIC "PFF0.0"

/**
 * Little-endian DDS file magic ("DDS ").
 */
#define DDS_MAGIC_L MKTAG('D', 'D', 'S', ' ')

#define VIDEO_FLAG_INTERMEDIATE     (1 << 0)
#define VIDEO_FLAG_COMPRESSED       (1 << 1)
#define VIDEO_FLAG_RLE              (1 << 2)
#define VIDEO_FLAG_TWO_PARTS        (1 << 3)

/**
 * Denominator of the time base for the video stream.
 * The actual time base is 1 / this value.
 */
#define VIDEO_TIMEBASE_DEN (1000000000)


typedef struct PFFDemuxContext
{
    //
    // Stuff related to video processing.
    //

    int                 video_stream_index;         /**< Index of the video output stream. < 0 if no video. */
    int                 first_video_section;        /**< Non-zero when the first video section is yet to be parsed */
    uint32_t            decompressed_size;          /**< Size of a decompressed video frame (without DDS header) */
    uint8_t             dds[124];                   /**< DDS header */
    AVBufferPool *      frame_pool;                 /**< Pool for allocating buffers for use in video decompression */
    AVBufferRef *       previous_frame;             /**< The previous frame we decoded */

    //
    // Stuff related to audio processing.
    //

    size_t              num_audio_streams;          /**< Number of audio streams in the file */
    char *              audio_languages;            /**< Multi-string of audio stream languages */
    FFFifoDemuxer **    audio_demuxer;              /**< Array of demuxers for the audio streams */
    int *               audio_stream_mapping;       /**< Mapping between a stream number in the file and an output FFmpeg stream */

    //
    // General stuff.
    //

    int                 new_frame;                  /**< Are we expecting a new frame? */
    double              current_frame_timestamp;    /**< Timestamp of the frame currently being processed */
    int                 current_audio_track;        /**< Index of the audio track we're currently processing */
} PFFDemuxContext;


/**
 * The OGG demuxer we'll be using for parsing SOUND_VORBIS data.
 */
extern const AVInputFormat ff_ogg_demuxer;


/**
 * Determines whether the given buffer is the beginning of a PFF file.
 *
 * @return Probe score
 */
static int pff_probe(AVProbeData const * p)
{
    if (NULL == p)
    {
        return 0;
    }

    if (NULL == p->buf || p->buf_size < sizeof(PFF_MAGIC))
    {
        return 0;
    }

    if (0 != memcmp(PFF_MAGIC, p->buf, sizeof(PFF_MAGIC)))
    {
        return 0;
    }

    return AVPROBE_SCORE_MAX;
}

/**
 * Reads the PFF header and initializes the demuxing state.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_read_header(AVFormatContext * s)
{
    int                 result                  = AVERROR_BUG;
    PFFDemuxContext *   context                 = NULL;
    char                scratch[100]            = { '\0' };
    AVStream *          stream                  = NULL;
    size_t              current_language_len    = 0;
    size_t              current_language_size   = 0;
    size_t              audio_languages_size    = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != s->pb);

    context = (PFFDemuxContext *)(s->priv_data);
    memset(context, 0, sizeof(*context));

    // Set AVFMTCTX_NOHEADER so that we can defer creation of the audio streams
    // until we see the first audio packet, and get a chance to initialize the OGG
    // demuxer.
    // Because of all those shenanigans we alsoo set AVFMTCTX_UNSEEKABLE. It's just
    // too much of a pain.
    s->ctx_flags |= AVFMTCTX_NOHEADER | AVFMTCTX_UNSEEKABLE;

    (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
    if (0 != strcmp(PFF_MAGIC, scratch))
    {
        av_log(s, AV_LOG_FATAL, "Missing PFF header magic\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    //
    // Parse video stream
    //

    (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));

    context->video_stream_index = -1;
    context->first_video_section = 1;
    if (av_strstart(scratch, "VIDEO_", NULL))
    {
        if (0 != strcmp("VIDEO_DDS", scratch))
        {
            av_log(s, AV_LOG_FATAL, "Video format %s is not supported\n", scratch);
            result = AVERROR_PATCHWELCOME;
            goto cleanup;
        }

        stream = avformat_new_stream(s, NULL);
        if (NULL == stream)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }
        stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        stream->codecpar->codec_id = AV_CODEC_ID_DDS;
        avpriv_set_pts_info(stream, 64, 1, VIDEO_TIMEBASE_DEN);

        context->video_stream_index = stream->index;

        (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
    }

    //
    // Parse audio streams
    //

    for (;;)
    {
        if (0 == strcmp("ENDHEADER", scratch))
        {
            break;
        }

        if (0 != strcmp("SOUND_VORBIS", scratch))
        {
            av_log(s, AV_LOG_FATAL, "Audio format %s is not supported\n", scratch);
            result = AVERROR_PATCHWELCOME;
            goto cleanup;
        }

        //
        // Read language
        //

        (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
        if (0 == strcmp("ENDHEADER", scratch))
        {
            av_log(s, AV_LOG_FATAL, "Missing audio stream language\n");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }

        current_language_len = strlen(scratch);
        current_language_size = (current_language_len + 1) * sizeof(scratch[0]);

        for (size_t i = 0; i < current_language_len; ++i)
        {
            scratch[i] = av_tolower(scratch[i]);
        }

        // Yes, this is quadratic, but there shouldn't be that many streams anyway
        result = av_reallocp(&context->audio_languages,
                             audio_languages_size + current_language_size);
        if (result < 0)
        {
            goto cleanup;
        }

        memmove(context->audio_languages + audio_languages_size,
                scratch,
                current_language_size);

        audio_languages_size += current_language_size;

        ++context->num_audio_streams;

        (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
    }

    if (0 != context->num_audio_streams)
    {
        context->audio_demuxer = av_calloc(context->num_audio_streams,
                                           sizeof(*(context->audio_demuxer)));
        if (NULL == context->audio_demuxer)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }

        context->audio_stream_mapping = av_calloc(context->num_audio_streams,
                                                  sizeof(*(context->audio_stream_mapping)));
        if (NULL == context->audio_stream_mapping)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }
        for (size_t i = 0; i < context->num_audio_streams; ++i)
        {
            context->audio_stream_mapping[i] = -1;
        }
    }

    context->new_frame = 1;
    context->current_frame_timestamp = NAN;
    context->current_audio_track = -1;

    result = 0;

cleanup:
    return result;
}

/**
 * Decodes Huffman-coded data into the given buffer.
 *
 * @param s         Context from whence the encoded data comes (s->pb).
 *                  Also used for logging.
 * @param buffer    Output buffer.
 * @param size      On entry, contains the output buffer size. On return, will hold
 *                  the size of the decoded data.
 *
 * @return 0 on success or an AVERROR value on failure.
 *
 * @remark On error, the input stream is left in an unspecified state.
 */
static int pff_decode_huffman(AVFormatContext * s, uint8_t * buffer, size_t * size)
{
    int         result      = AVERROR_BUG;
    uint16_t    tree_length = 0;
    uint16_t *  nodes       = NULL;
    uint32_t    mask        = 0;
    uint32_t    bits        = 0;
    uint16_t    index       = 0;
    uint16_t    value       = 0;
    size_t      decoded     = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != buffer);
    av_assert0(NULL != size);

    tree_length = avio_rl16(s->pb);
    if (0 == tree_length)
    {
        av_log(s, AV_LOG_FATAL, "Huffman tree is 0 length\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    nodes = av_malloc_array(tree_length, sizeof(*nodes));
    if (NULL == nodes)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    for (uint16_t i = 0; i < tree_length; ++i)
    {
        nodes[i] = avio_rl16(s->pb);
    }
    if (avio_feof(s->pb))
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading Huffman tree\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    for (;;)
    {
        if (0 == mask)
        {
            mask = 0x80000000;
            bits = avio_rl32(s->pb);
            if (avio_feof(s->pb))
            {
                av_log(s, AV_LOG_FATAL, "EOF while reading Huffman-coded data\n");
                result = AVERROR_EOF;
                goto cleanup;
            }
        }

        if (bits & mask)
        {
            index = nodes[index];
        }
        else
        {
            ++index;
        }
        if (index >= tree_length)
        {
            av_log(s, AV_LOG_FATAL, "Huffman tree index out of range\n");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }

        if (nodes[index] & 0x8000)
        {
            value = nodes[index] & 0x1FF;

            if (0x100 == value)
            {
                *size = decoded;
                break;
            }

            if (decoded >= *size)
            {
                av_log(s, AV_LOG_FATAL, "Huffman output buffer is too small\n");
                result = AVERROR_BUFFER_TOO_SMALL;
                goto cleanup;
            }

            buffer[decoded++] = value;
            index = 0;
        }

        mask >>= 1;
    }

    result = 0;

cleanup:
    av_freep(&nodes);

    return result;
}

/**
 * Decodes RLE data.
 *
 * @param input         Input buffer.
 * @param input_size    Input buffer size.
 * @param output        Output buffer.
 * @param output_size   Output buffer size.
 * @param written       If non-NULL, will receive the size of the written data.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_decode_rle(uint8_t const * input,
                          size_t input_size,
                          uint8_t * output,
                          size_t output_size,
                          size_t * written)
{
    int             result          = AVERROR_BUG;
    uint8_t const * current_input   = NULL;
    uint8_t *       current_output  = NULL;
    size_t          to_write        = 0;

    av_assert0(NULL != input);
    av_assert0(NULL != output);

    current_input = input;
    current_output = output;
    while (current_input < input + input_size)
    {
        to_write = *current_input & 0x7F;

        if (current_output + to_write > output + output_size)
        {
            result = AVERROR_BUFFER_TOO_SMALL;
            goto cleanup;
        }

        if (*current_input & 0x80)
        {
            memset(current_output, 0, to_write);
            ++current_input;
        }
        else
        {
            memmove(current_output, current_input + 1, to_write);
            current_input += 1 + to_write;
        }

        current_output += to_write;
    }

    if (NULL != written)
    {
        *written = current_output - output;
    }

    result = 0;

cleanup:
    return result;
}

/**
 * Interleaves an array of uint32_t.
 *
 * @param s         Context for logging.
 * @param buffer    Input buffer.
 * @param size      Input buffer size. Must be non-zero and divislbe by 8.
 * @param output    Output buffer. Must the at least as large as the input buffer.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_interleave(AVFormatContext * s, uint8_t * buffer, size_t size, uint8_t * output)
{
    int         result      = AVERROR_BUG;
    uint8_t *   first_part  = NULL;
    uint8_t *   second_part = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != buffer);
    av_assert0(NULL != output);

    // TODO: https://cs.stackexchange.com/q/332

    // We should be able to split the buffer in two halves, and each half must
    // have a whole, non-zero amount of dwords
    if (size < 8 || 0 != size % 8)
    {
        av_log(s, AV_LOG_FATAL, "Buffer is too small for interleaving\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    for (first_part = buffer, second_part = buffer + size / 2;
         second_part < buffer + size;
         first_part += 4, second_part += 4, output += 8)
    {
        AV_WL32(output, AV_RL32(first_part));
        AV_WL32(output + 4, AV_RL32(second_part));
    }

    result = 0;

cleanup:
    return result;
}

/**
 * XORs two buffers.
 *
 * @param first     Buffer to XOR.
 * @param second    Buffer to XOR with.
 * @param size      Size of the buffers.
 */
static void pff_xor(uint8_t * first, uint8_t const * second, size_t size)
{
    av_assert0(NULL != first);
    av_assert0(NULL != second);

    for (size_t index = 0; index < size; ++index)
    {
        first[index] ^= second[index];
    }
}

/**
 * Processes a PFF video section and extracts a single video frame from it.
 *
 * @param s     Demuxing context.
 * @param pkt   Packet to fill.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_process_video_section(AVFormatContext * s, AVPacket * pkt)
{
    int                 result                  = AVERROR_BUG;
    PFFDemuxContext *   context                 = NULL;
    uint8_t             flags                   = 0;
    AVBufferRef *       frame_data              = NULL;
    size_t              frame_size              = 0;
    AVBufferRef *       frame_temp              = NULL;
    size_t              huffman_decoded_size    = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != pkt);

    context = (PFFDemuxContext *)(s->priv_data);

    if (context->video_stream_index < 0)
    {
        av_log(s, AV_LOG_FATAL, "Encountered video frame, but video not declared in file header\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    av_assert0(!isnan(context->current_frame_timestamp));

    avio_rl32(s->pb);   // Section size
    if (avio_feof(s->pb))
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading video section size\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    if (context->first_video_section)
    {
        context->first_video_section = 0;

        context->decompressed_size = avio_rl32(s->pb);
        if (avio_feof(s->pb))
        {
            av_log(s, AV_LOG_FATAL, "EOF while reading decompressed frame size\n");
            result = AVERROR_EOF;
            goto cleanup;
        }

        if (DDS_MAGIC_L != avio_rl32(s->pb))
        {
            av_log(s, AV_LOG_FATAL, "Missing DDS magic\n");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }

        result = avio_read(s->pb, context->dds, sizeof(context->dds));
        if (result < 0)
        {
            goto cleanup;
        }
        if (result != sizeof(context->dds))
        {
            av_log(s, AV_LOG_FATAL, "EOF while reading DDS header\n");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }
        if (!(AV_RL32(&context->dds[0x4C]) & 0x4))  // !(ddspf.dwFlags & DDPF_FOURCC)
        {
            av_log(s, AV_LOG_WARNING, "Missing DDPF_FOURCC flag in DDS header\n");
            // Might work, who knows.
        }
        if (MKBETAG('D', 'X', 'T', '1') != AV_RB32(&context->dds[0x50]))  // ddspf.dwFourCC
        {
            av_log(s, AV_LOG_WARNING, "Unexpected FourCC: 0x%08" PRIX32 "\n", AV_RB32(&context->dds[0x50]));
            // Might work, who knows.
        }

        context->frame_pool = av_buffer_pool_init(context->decompressed_size, NULL);
        if (NULL == context->frame_pool)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }
    }

    flags = avio_r8(s->pb);
    if (avio_feof(s->pb))
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading video frame flags\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    if (!(flags & VIDEO_FLAG_COMPRESSED))
    {
        av_log(s, AV_LOG_FATAL, "Uncompressed video frames are not supported\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    frame_data = av_buffer_pool_get(context->frame_pool);
    if (NULL == frame_data)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }
    av_assert0(av_buffer_is_writable(frame_data));

    huffman_decoded_size = context->decompressed_size;
    result = pff_decode_huffman(s, frame_data->data, &huffman_decoded_size);
    if (result < 0)
    {
        goto cleanup;
    }
    frame_size = huffman_decoded_size;

    if ((flags & VIDEO_FLAG_TWO_PARTS) && !(flags & VIDEO_FLAG_RLE))
    {
        huffman_decoded_size = context->decompressed_size - huffman_decoded_size;
        result = pff_decode_huffman(s,
                                    frame_data->data + frame_size,
                                    &huffman_decoded_size);
        if (result < 0)
        {
            goto cleanup;
        }
        frame_size += huffman_decoded_size;
    }

    if (flags & VIDEO_FLAG_RLE)
    {
        frame_temp = av_buffer_pool_get(context->frame_pool);
        if (NULL == frame_temp)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }
        av_assert0(av_buffer_is_writable(frame_temp));

        result = pff_decode_rle(frame_data->data,
                                frame_size,
                                frame_temp->data,
                                context->decompressed_size,
                                &frame_size);
        if (result < 0)
        {
            goto cleanup;
        }

        av_buffer_unref(&frame_data);
        frame_data = frame_temp;
        frame_temp = NULL;
    }

    if (frame_size != context->decompressed_size)
    {
        av_log(s, AV_LOG_FATAL, "Frame size is smaller than expected\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    if (flags & VIDEO_FLAG_TWO_PARTS)
    {
        frame_temp = av_buffer_pool_get(context->frame_pool);
        if (NULL == frame_temp)
        {
            result = AVERROR(ENOMEM);
            goto cleanup;
        }
        av_assert0(av_buffer_is_writable(frame_temp));

        result = pff_interleave(s,
                                frame_data->data,
                                context->decompressed_size,
                                frame_temp->data);
        if (result < 0)
        {
            goto cleanup;
        }

        av_buffer_unref(&frame_data);
        frame_data = frame_temp;
        frame_temp = NULL;
    }

    if (flags & VIDEO_FLAG_INTERMEDIATE)
    {
        if (NULL == context->previous_frame)
        {
            av_log(s, AV_LOG_FATAL, "First frame is intermediate\n");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }

        pff_xor(frame_data->data,
                context->previous_frame->data,
                context->decompressed_size);
    }

    result = av_new_packet(pkt, context->decompressed_size + sizeof(context->dds) + 4);
    if (result < 0)
    {
        goto cleanup;
    }

    AV_WL32(pkt->data, DDS_MAGIC_L);
    memmove(pkt->data + 4, context->dds, sizeof(context->dds));
    memmove(pkt->data + 4 + sizeof(context->dds), frame_data->data, context->decompressed_size);

    pkt->stream_index = context->video_stream_index;

    pkt->dts = AV_NOPTS_VALUE;
    pkt->pts = context->current_frame_timestamp * VIDEO_TIMEBASE_DEN;

    if (!(flags & VIDEO_FLAG_INTERMEDIATE))
    {
        pkt->flags |= AV_PKT_FLAG_KEY;
    }

    av_buffer_unref(&context->previous_frame);
    context->previous_frame = frame_data;
    frame_data = NULL;

    result = 0;

cleanup:
    av_buffer_unref(&frame_temp);
    av_buffer_unref(&frame_data);

    return result;
}

/**
 * Callback for reading audio data into a FIFO demuxer.
 *
 * @param opaque    Pointer to an AVFormatContext.
 * @param buffer    Output buffer.
 * @param size      On entry, the size of the output buffer. On return, holds the
 *                  size of the read data.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_read_audio(void * opaque, void * buffer, size_t * size)
{
    int                 result  = AVERROR_BUG;
    AVFormatContext *   s       = (AVFormatContext *)opaque;

    av_assert0(NULL != opaque);
    av_assert0(NULL != buffer);
    av_assert0(NULL != size);

    av_assert0(NULL != s->pb);

    result = avio_read(s->pb, buffer, FFMIN(*size, INT_MAX));
    if (result < 0)
    {
        goto cleanup;
    }
    *size = result;

    result = 0;

cleanup:
    return result;
}

/**
 * Creates a new AVStream with the same parameters as an existing stream.
 *
 * @param new_stream        Will receive the created stream.
 * @param s                 Context for the new stream.
 * @param source_stream     Stream to copy parameters from.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
static int pff_create_stream_from_stream(AVStream ** new_stream,
                                         AVFormatContext * s,
                                         AVStream * source_stream)
{
    int result  = AVERROR_BUG;

    av_assert0(NULL != new_stream);
    av_assert0(NULL != s);
    av_assert0(NULL != source_stream);

    *new_stream = avformat_new_stream(s, NULL);
    if (NULL == *new_stream)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    // Adapted from copy_stream_props in concatdec.c

    av_assert0(AV_CODEC_ID_NONE == (*new_stream)->codecpar->codec_id);

    if (!source_stream->codecpar->codec_id)
    {
        if ((*new_stream)->codecpar->extradata_size < source_stream->codecpar->extradata_size)
        {
            result = ff_alloc_extradata((*new_stream)->codecpar,
                                        source_stream->codecpar->extradata_size);
            if (result < 0)
            {
                goto cleanup;
            }
        }

        if (NULL != (*new_stream)->codecpar->extradata &&
            NULL != source_stream->codecpar->extradata)
        {
            memmove((*new_stream)->codecpar->extradata,
                    source_stream->codecpar->extradata,
                    source_stream->codecpar->extradata_size);
        }

        result = 0;
        goto cleanup;
    }

    result = avcodec_parameters_copy((*new_stream)->codecpar, source_stream->codecpar);
    if (result < 0)
    {
        goto cleanup;
    }

    (*new_stream)->r_frame_rate        = source_stream->r_frame_rate;
    (*new_stream)->avg_frame_rate      = source_stream->avg_frame_rate;
    (*new_stream)->sample_aspect_ratio = source_stream->sample_aspect_ratio;

    avpriv_set_pts_info(*new_stream,
                        64,
                        source_stream->time_base.num,
                        source_stream->time_base.den);

    result = av_dict_copy(&(*new_stream)->metadata, source_stream->metadata, 0);
    if (result < 0)
    {
        goto cleanup;
    }

    result = ff_stream_side_data_copy(*new_stream, source_stream);
    if (result < 0)
    {
        goto cleanup;
    }

    result = 0;

cleanup:
    if (result < 0 && NULL != *new_stream)
    {
        ff_free_stream(s, *new_stream);
        *new_stream = NULL;
    }

    return result;
}

/**
 * Attempts to read an audio packet from the current audio track.
 *
 * @param s     Demuxing context.
 * @param pkt   Packet to fill.
 *
 * @return  0 on success or an AVERROR value on failure. AVERROR(EAGAIN) is returned
 *          when no audio data is currently available.
 */
static int pff_try_read_audio_frame(AVFormatContext * s, AVPacket * pkt)
{
    int                 result              = AVERROR_BUG;
    PFFDemuxContext *   context             = NULL;
    AVStream **         input_streams       = NULL;
    unsigned int        num_input_streams   = 0;
    enum AVCodecID      codec_id            = AV_CODEC_ID_NONE;
    AVStream *          new_stream          = NULL;
    char *              language            = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != pkt);

    context = (PFFDemuxContext *)(s->priv_data);

    if (0 > context->current_audio_track)
    {
        result = AVERROR(EAGAIN);
        goto cleanup;
    }

    result = ff_fifo_demuxer_read_frame(context->audio_demuxer[context->current_audio_track], pkt);
    if (AVERROR_EOF == result)
    {
        // This is OGG, so EOF means there's no more data available, not that
        // there *can't* be any more data (because we reached some footer,
        // for instance).
        // Tell the caller to try again later.
        result = AVERROR(EAGAIN);
    }
    if (result < 0)
    {
        goto cleanup;
    }

    result = ff_fifo_demuxer_get_streams(context->audio_demuxer[context->current_audio_track],
                                         &input_streams,
                                         &num_input_streams);
    if (result < 0)
    {
        goto cleanup;
    }

    if (1 < num_input_streams)
    {
        av_log(s, AV_LOG_FATAL, "Audio tracks with more than 1 stream are not supported\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    if (pkt->stream_index < 0)
    {
        av_log(s, AV_LOG_WARNING, "Negative stream index in audio packet\n");
        result = 0;
        goto cleanup;
    }

    av_assert0(NULL != input_streams);
    av_assert0(0 < num_input_streams);

    codec_id = input_streams[pkt->stream_index]->codecpar->codec_id;

    if (AV_CODEC_ID_VORBIS != codec_id)
    {
        av_log(s, AV_LOG_WARNING, "Non-Vorbis audio track (codec ID: %d)\n", codec_id);
        // Might work. Let's try.
    }

    if (context->audio_stream_mapping[context->current_audio_track] < 0)
    {
        result = pff_create_stream_from_stream(&new_stream, s, input_streams[pkt->stream_index]);
        if (result < 0)
        {
            goto cleanup;
        }

        language = context->audio_languages;
        for (int i = 0; i < context->current_audio_track; ++i)
        {
            language += strlen(language) + 1;
        }

        // If language is invalid, ff_convert_lang_to will return NULL,
        // and av_dict_set will do nothing.
        // Also see: https://ffmpeg.org/doxygen/trunk/asfdec__o_8c_source.html#l00706
        result = av_dict_set(&new_stream->metadata,
                             "language",
                             ff_convert_lang_to(language, AV_LANG_ISO639_2_BIBL),
                             0);
        if (result < 0)
        {
            goto cleanup;
        }

        context->audio_stream_mapping[context->current_audio_track] = new_stream->index;

        new_stream = NULL;
    }

    pkt->stream_index = context->audio_stream_mapping[context->current_audio_track];

    result = 0;

cleanup:
    if (NULL != new_stream)
    {
        ff_free_stream(s, new_stream);
        new_stream = NULL;
    }

    return result;
}

/**
 * Processes a PFF sound section and attempts to extract a single audio frame from it.
 *
 * @param s     Demuxing context.
 * @param pkt   Packet to fill.
 *
 * @return  0 on success or an AVERROR value on failure. AVERROR(EAGAIN) is returned
 *          when no audio data is currently available.
 */
static int pff_process_sound_section(AVFormatContext * s, AVPacket * pkt)
{
    int                 result                  = AVERROR_BUG;
    PFFDemuxContext *   context                 = NULL;
    uint32_t            section_size            = 0;
    uint8_t             track_index             = 0;
    size_t              audio_data_read         = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != pkt);

    context = (PFFDemuxContext *)(s->priv_data);

    section_size = avio_rl32(s->pb);
    if (avio_feof(s->pb))
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading audio section size\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    track_index = avio_r8(s->pb);
    if (avio_feof(s->pb))
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading audio track index\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    if (track_index >= context->num_audio_streams)
    {
        av_log(s, AV_LOG_FATAL, "Encountered audio frame that was not declared in the file header\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    if (NULL == context->audio_demuxer[track_index])
    {
        result = ff_alloc_fifo_demuxer(&context->audio_demuxer[track_index],
                                       section_size - 1,
                                       &ff_ogg_demuxer);
        if (result < 0)
        {
            goto cleanup;
        }
    }

    result = ff_fifo_demuxer_ensure_write_buffer(context->audio_demuxer[track_index],
                                                 section_size - 1);
    if (result < 0)
    {
        goto cleanup;
    }

    audio_data_read = section_size - 1;
    result = ff_fifo_demuxer_write_from_cb(context->audio_demuxer[track_index],
                                           &pff_read_audio,
                                           s,
                                           &audio_data_read);
    if (result < 0)
    {
        goto cleanup;
    }
    if (audio_data_read != section_size - 1)
    {
        av_log(s, AV_LOG_FATAL, "EOF while reading audio data\n");
        result = AVERROR_EOF;
        goto cleanup;
    }

    context->current_audio_track = track_index;
    result = pff_try_read_audio_frame(s, pkt);
    if (AVERROR(EAGAIN) == result)
    {
        context->current_audio_track = -1;
    }
    if (result < 0)
    {
        goto cleanup;
    }

    result = 0;

cleanup:
    return result;
}

/**
 * Tries to read the next packet from the file.
 *
 * @param s     Demuxing context.
 * @param pkt   Packet to fill.
 *
 * @return  0 on success or an AVERROR value on failure. AVERROR(EAGAIN) is returned
 *          if the operation should be immediately retried.
 */
static int pff_try_read_packet(AVFormatContext * s, AVPacket * pkt)
{
    int                 result          = AVERROR_BUG;
    PFFDemuxContext *   context         = NULL;
    char                scratch[100]    = { '\0' };

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != pkt);

    if (s->io_repositioned)
    {
        av_log(s, AV_LOG_FATAL, "You reposition miette?\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    context = (PFFDemuxContext *)(s->priv_data);

    // First, try to read any audio frames we have.

    result = pff_try_read_audio_frame(s, pkt);
    if (AVERROR(EAGAIN) == result)
    {
        context->current_audio_track = -1;
    }
    else
    {
        goto cleanup;
    }

    // No more audio. Time to dig further into the file.

    (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));

    for (;;)
    {
        if (context->new_frame)
        {
            if (0 == strcmp("ENDFILE", scratch))
            {
                result = AVERROR_EOF;
                goto cleanup;
            }

            if (0 != strcmp("FRAME", scratch))
            {
                av_log(s, AV_LOG_FATAL, "Missing frame header magic\n");
                result = AVERROR_INVALIDDATA;
                goto cleanup;
            }

            context->new_frame = 0;

            // Frame size
            avio_rl32(s->pb);

            context->current_frame_timestamp = ((av_alias64){.u64 = avio_rl64(s->pb)}).f64;
            if (isnan(context->current_frame_timestamp))
            {
                av_log(s, AV_LOG_FATAL, "Frame timestamp is NAN\n");
                result = AVERROR_INVALIDDATA;
                goto cleanup;
            }

            if (avio_feof(s->pb))
            {
                av_log(s, AV_LOG_FATAL, "EOF while reading frame header\n");
                result = AVERROR_EOF;
                goto cleanup;
            }

            (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
        }

        if (0 != strcmp("ENDFRAME", scratch))
        {
            break;
        }

        context->new_frame = 1;
        context->current_frame_timestamp = NAN;
        av_assert0(0 > context->current_audio_track);;

        (void)avio_get_str(s->pb, sizeof(scratch), scratch, sizeof(scratch));
    }

    if (0 == strcmp("VIDEO", scratch))
    {
        result = pff_process_video_section(s, pkt);
    }
    else if (0 == strcmp("SOUND", scratch))
    {
        result = pff_process_sound_section(s, pkt);
    }
    else
    {
        av_log(s, AV_LOG_FATAL, "Frame section type %s is not supported\n", scratch);
        result = AVERROR_PATCHWELCOME;
    }

    // Keep last status

cleanup:
    return result;
}

/**
 * Reads the next packet from the file.
 *
 * @param s     Demuxing context.
 * @param pkt   Packet to fill.
 *
 * @return  0 on success or an AVERROR value on failure.
 */
static int pff_read_packet(AVFormatContext * s, AVPacket * pkt)
{
    int result  = AVERROR_BUG;

    while (AVERROR(EAGAIN) == (result = pff_try_read_packet(s, pkt)))
    {
    }

    return result;
}

/**
 * Destroys the demuxing context.
 *
 * @param s     Demuxing context.
 *
 * @return 0
 */
static int pff_read_close(AVFormatContext * s)
{
    PFFDemuxContext *   context = NULL;

    if (NULL == s || NULL == s->priv_data)
    {
        return 0;
    }

    context = (PFFDemuxContext *)(s->priv_data);

    av_freep(&context->audio_stream_mapping);
    if (NULL != context->audio_demuxer)
    {
        for (size_t i = 0; i < context->num_audio_streams; ++i)
        {
            ff_fifo_demuxer_free(&context->audio_demuxer[i]);
        }
    }
    av_freep(&context->audio_demuxer);
    av_freep(&context->audio_languages);
    context->num_audio_streams = 0;

    av_buffer_unref(&context->previous_frame);
    av_buffer_pool_uninit(&context->frame_pool);

    return 0;
}


const AVInputFormat ff_pff_demuxer = {
    .name           = "pff",
    .long_name      = NULL_IF_CONFIG_SMALL("Still Life 2 PFF"),
    .extensions     = "pff",
    .flags          = AVFMT_GENERIC_INDEX | AVFMT_NOBINSEARCH | AVFMT_NOGENSEARCH |
                      AVFMT_NO_BYTE_SEEK,

    .read_probe     = &pff_probe,
    .read_header    = &pff_read_header,
    .read_packet    = &pff_read_packet,
    .read_close     = &pff_read_close,
    .priv_data_size = sizeof(PFFDemuxContext),
    .flags_internal = FF_FMT_INIT_CLEANUP,
};
