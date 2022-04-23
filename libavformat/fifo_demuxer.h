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

#ifndef AVFORMAT_FIFO_DEMUXER_H
#define AVFORMAT_FIFO_DEMUXER_H

#include <stddef.h>

#include "libavcodec/packet.h"

#include "avformat.h"


/**
 * FIFO demuxer.
 *
 * Pairs an *input* AVFormatContext with a FIFO, to enable gradually feeding data into
 * the demuxer while also reading packets from it.
 */
typedef struct FFFifoDemuxer FFFifoDemuxer;


/**
 * Callback for feeding data *into* the FIFO.
 *
 * @param opaque    Caller-supplied arbitrary pointer.
 * @param buffer    Buffer to read into.
 * @param size      On entry, the size of the output buffer. On return, holds the
 *                  size of the read data.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
typedef int FFFifoDemuxerCB(void * opaque, void * buffer, size_t * size);


/**
 * Creates a new FIFO demuxer.
 *
 * @param pdemuxer      Will receive an allocated demuxer.
 * @param initial_size  Initial capacity of the FIFO.
 * @param input_format  Optional input format for the demuxer. If NULL,
 *                      the format will be auto-detected.
 *
 * @return 0 on success or an AVERROR value on failure.
 *
 * @remark  No data will be read from the FIFO here. Demuxer initialization
 *          will happen at the first call to ff_fifo_demuxer_read_frame.
 */
int ff_alloc_fifo_demuxer(FFFifoDemuxer ** pdemuxer,
                          size_t initial_size,
                          AVInputFormat const * input_format);

/**
 * Frees a FIFO demuxer and sets the pointer to NULL.
 */
void ff_fifo_demuxer_free(FFFifoDemuxer ** pdemuxer);

/**
 * Ensures that at least size bytes can be written into the FIFO.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
int ff_fifo_demuxer_ensure_write_buffer(FFFifoDemuxer * demuxer, size_t size);

/**
 * Feeds data into the FIFO using a callback.
 *
 * @param demuxer   FIFO demuxer to feed data into.
 * @param read_cb   Callback for filling the FIFO.
 * @param opaque    Arbitrary pointer.
 * @param size      On input, specifies the maximum number of bytes to read using
 *                  the callback. On output, will receive the number of bytes
 *                  actually read.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
int ff_fifo_demuxer_write_from_cb(FFFifoDemuxer * demuxer,
                                  FFFifoDemuxerCB * read_cb,
                                  void * opaque,
                                  size_t * size);

/**
 * Reads a frame from the demuxer.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
int ff_fifo_demuxer_read_frame(FFFifoDemuxer * demuxer, AVPacket * packet);

/**
 * Obtains the AVStream array from the internal AVFormatContext.
 *
 * @param demuxer       FIFO demuxer.
 * @param streams       If not NULL, will receive a pointer to the streams array.
 * @param num_streams   If not NULL, will receive the number of streams.
 *
 * @return 0 on success or an AVERROR value on failure.
 */
int ff_fifo_demuxer_get_streams(FFFifoDemuxer * demuxer,
                                AVStream *** streams,
                                unsigned int * num_streams);

#endif /* AVFORMAT_FIFO_DEMUXER_H */
