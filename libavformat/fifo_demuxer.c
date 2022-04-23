/*
 * FIFO demuxer
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

#include "fifo_demuxer.h"

#include <stddef.h>

#include "libavutil/fifo.h"
#include "libavutil/error.h"
#include "libavutil/mem.h"
#include "libavutil/avassert.h"
#include "libavutil/log.h"
#include "libavutil/macros.h"

#include "avformat.h"
#include "internal.h"
#include "avio.h"


#define READER_BUFFER_SIZE (4096)


struct FFFifoDemuxer
{
    int                     open;

    AVInputFormat const *   input_format;

    AVFifo *                fifo;
    AVIOContext *           fifo_reader;
    AVFormatContext *       demuxer;
};


static int fifo_read(void * opaque, uint8_t * buffer, int size)
{
    int         result      = AVERROR_BUG;
    AVFifo *    fifo        = (AVFifo *)opaque;
    size_t      can_read    = 0;
    int         to_read     = 0;

    av_assert0(NULL != opaque);
    av_assert0(NULL != buffer);
    av_assert0(size >= 0);

    can_read = av_fifo_can_read(fifo);

    to_read = (int)FFMIN((size_t)size, can_read);

    if (0 == to_read)
    {
        return AVERROR_EOF;
    }

    result = av_fifo_read(fifo, buffer, to_read);
    if (result < 0)
    {
        return result;
    }

    return to_read;
}

int ff_alloc_fifo_demuxer(FFFifoDemuxer ** pdemuxer,
                          size_t initial_size,
                          AVInputFormat const * input_format)
{
    int                 result          = AVERROR_BUG;
    FFFifoDemuxer *     demuxer         = NULL;
    AVFifo *            fifo            = NULL;
    void *              reader_buf      = NULL;
    AVIOContext *       reader          = NULL;
    AVFormatContext *   actual_demuxer  = NULL;

    if (NULL == pdemuxer)
    {
        result = AVERROR(EINVAL);
        goto cleanup;
    }

    demuxer = av_calloc(1, sizeof(*demuxer));
    if (NULL == demuxer)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    demuxer->input_format = input_format;

    fifo = av_fifo_alloc2(initial_size, 1, 0);
    if (NULL == fifo)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    reader_buf = av_malloc(READER_BUFFER_SIZE);
    if (NULL == reader_buf)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    reader = avio_alloc_context(reader_buf,
                                READER_BUFFER_SIZE,
                                0,
                                fifo,
                                &fifo_read,
                                NULL,
                                NULL);
    if (NULL == reader)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }
    reader_buf = NULL;

    actual_demuxer = avformat_alloc_context();
    if (NULL == actual_demuxer)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    actual_demuxer->pb = reader;

    // Transfer ownership:
    demuxer->fifo = fifo;
    fifo = NULL;
    demuxer->fifo_reader = reader;
    reader = NULL;
    demuxer->demuxer = actual_demuxer;
    actual_demuxer = NULL;
    *pdemuxer = demuxer;
    demuxer = NULL;

    result = 0;

cleanup:
    if (NULL != actual_demuxer)
    {
        avformat_free_context(actual_demuxer);
        actual_demuxer = NULL;
    }
    if (NULL != reader)
    {
        av_freep(&reader->buffer);
    }
    avio_context_free(&reader);
    av_freep(&reader_buf);
    av_fifo_freep2(&fifo);
    av_freep(&demuxer);

    return result;
}

void ff_fifo_demuxer_free(FFFifoDemuxer ** pdemuxer)
{
    FFFifoDemuxer * demuxer = NULL;

    if (NULL == pdemuxer || NULL == *pdemuxer)
    {
        return;
    }

    demuxer = *pdemuxer;
    *pdemuxer = NULL;

    if (demuxer->open)
    {
        avformat_close_input(&demuxer->demuxer);
    }
    else if (NULL != demuxer->demuxer)
    {
        avformat_free_context(demuxer->demuxer);
        demuxer->demuxer = NULL;
    }

    if (NULL != demuxer->fifo_reader)
    {
        av_freep(&demuxer->fifo_reader->buffer);
    }
    avio_context_free(&demuxer->fifo_reader);

    av_fifo_freep2(&demuxer->fifo);

    av_freep(&demuxer);
}

int ff_fifo_demuxer_ensure_write_buffer(FFFifoDemuxer * demuxer, size_t size)
{
    int     result      = AVERROR_BUG;
    size_t  can_write   = 0;

    if (NULL == demuxer)
    {
        result = AVERROR(EINVAL);
        goto cleanup;
    }

    can_write = av_fifo_can_write(demuxer->fifo);

    if (can_write >= size)
    {
        result = 0;
        goto cleanup;
    }

    // Allocate a little more
    result = av_fifo_grow2(demuxer->fifo, (size - can_write) * 2);

    // Keep last status

cleanup:
    return result;
}

int ff_fifo_demuxer_write_from_cb(FFFifoDemuxer * demuxer,
                                  FFFifoDemuxerCB * read_cb,
                                  void * opaque,
                                  size_t * size)
{
    int result  = AVERROR_BUG;

    if (NULL == demuxer || NULL == read_cb || NULL == size)
    {
        result = AVERROR(EINVAL);
        goto cleanup;
    }

    result = av_fifo_write_from_cb(demuxer->fifo, read_cb, opaque, size);

    // Keep last status

cleanup:
    return result;
}

int ff_fifo_demuxer_read_frame(FFFifoDemuxer * demuxer, AVPacket * packet)
{
    int result  = AVERROR_BUG;

    if (NULL == demuxer || NULL == packet)
    {
        result = AVERROR(EINVAL);
        goto cleanup;
    }

    if (!demuxer->open)
    {
        result = avformat_open_input(&demuxer->demuxer, NULL, demuxer->input_format, NULL);
        if (result < 0)
        {
            av_log(NULL, AV_LOG_FATAL, "Failed opening decoder\n");
            goto cleanup;
        }
        demuxer->open = 1;
    }

    // Reset the EOF state in the IO reader, so that any new data
    // in the FIFO will actually be processed
    demuxer->fifo_reader->eof_reached = 0;

    result = av_read_frame(demuxer->demuxer, packet);

    // Keep last status

cleanup:
    return result;
}

int ff_fifo_demuxer_get_streams(FFFifoDemuxer * demuxer,
                                AVStream *** streams,
                                unsigned int * num_streams)
{
    if (NULL == demuxer)
    {
        return AVERROR(EINVAL);
    }

    if (NULL != streams)
    {
        *streams = demuxer->demuxer->streams;
    }

    if (NULL != num_streams)
    {
        *num_streams = demuxer->demuxer->nb_streams;
    }

    return 0;
}
