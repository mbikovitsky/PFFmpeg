/*
 * Still Life 2 PFF muxer
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

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "libavutil/internal.h"
#include "libavutil/error.h"
#include "libavutil/avassert.h"
#include "libavutil/log.h"
#include "libavutil/avutil.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/macros.h"
#include "libavutil/mem.h"
#include "libavutil/rational.h"

#include "libavcodec/codec_id.h"
#include "libavcodec/put_bits.h"
#include "libavcodec/get_bits.h"
#include "libavcodec/defs.h"

#include "avformat.h"
#include "internal.h"
#include "avio.h"


#define PFF_HEADER          "PFF0.0\0VIDEO_DDS\0ENDHEADER"
#define PFF_TRAILER         "ENDFILE"
#define FRAME_MAGIC         "FRAME"
#define VIDEO_SECTION_MAGIC "VIDEO"
#define END_FRAME_MAGIC     "ENDFRAME"

#define VIDEO_FLAG_INTERMEDIATE     (1 << 0)
#define VIDEO_FLAG_COMPRESSED       (1 << 1)
#define VIDEO_FLAG_RLE              (1 << 2)
#define VIDEO_FLAG_TWO_PARTS        (1 << 3)

/**
 * Maximum number of symbols in a stream for Huffman coding.
 *
 * 256 for all possible byte values + 1 for the end-of-stream sentinel.
 */
#define MAX_HUFFMAN_SYMBOLS (256 + 1)

/**
 * Sentinel symbol for marking the end of Huffman-coded data.
 */
#define HUFFMAN_SENTINEL_SYMBOL (0x100)


/**
 * Returns the size of an array.
 *
 * The result is undefined if used on anything other than an array.
 */
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))


/**
 * A node in a Huffman tree.
 */
typedef struct HuffmanNode
{
    uint16_t                value;  /**< Value of the node. Valid only for leaves. */
    size_t                  weight; /**< Weight of the node. */
    struct HuffmanNode *    left;   /**< Left child. */
    struct HuffmanNode *    right;  /**< Right child. */
    struct HuffmanNode *    parent; /**< Parent node. */

    // For serialization
    uint16_t    parent_index;       /**< Index of the parent node in the serialized tree. */
} HuffmanNode;

/**
 * Represents the Huffman bit code of a single symbol.
 */
typedef struct HuffmanCode
{
    uint16_t    value;  /**< Symbol this code is for. */

    // Large enough to store 257 bits
    uint8_t     code[33 + AV_INPUT_BUFFER_PADDING_SIZE];    /**< The actual bit code. */

    uint16_t    bit_length; /**< Length of the code, in bits. */
} HuffmanCode;

typedef struct PFFMuxContext
{
    int         first_frame;        /**< Non-zero when we're expecting the first video frame. */
    uint8_t     dds[0x80];          /**< The DDS header all frames should have. */
    uint32_t    decompressed_size;  /**< Size of the decompressed pixel data for each frame. */

    //
    // Stuff for the Huffman encoder.
    //

    HuffmanNode     nodes[2 * MAX_HUFFMAN_SYMBOLS - 1]; /**< Storage for the nodes of the tree. */
    uint16_t        num_symbols;                        /**< Number of symbols (aka leaves) in the current tree. */
    HuffmanNode *   root;                               /**< Root of the current tree. */
    HuffmanCode     codes[MAX_HUFFMAN_SYMBOLS];         /**< Codes extracted from the current tree. */
} PFFMuxContext;


/**
 * Reverses a byte array in-place.
 */
static void pff_reverse_byte_array(uint8_t * array, size_t size)
{
    uint8_t *   first   = NULL;
    uint8_t *   last    = NULL;

    // Based on https://github.com/llvm-mirror/libcxx/blob/a12cb9d211019d99b5875b6d8034617cbc24c2cc/include/algorithm#L2259-L2291

    if (NULL == array)
    {
        return;
    }

    first = array;
    last = array + size;

    while (first != last)
    {
        --last;
        if (first == last)
        {
            break;
        }

        FFSWAP(uint8_t, *first, *last);

        ++first;
    }
}

/**
 * Comparison function for HuffmanNode structs, comparing by weight
 * in descending order.
 */
static int pff_huffman_cmp_node_weight_desc(void const * va, void const * vb)
{
    HuffmanNode const * a   = va;
    HuffmanNode const * b   = vb;

    av_assert0(NULL != va);
    av_assert0(NULL != vb);

    return -FFDIFFSIGN(a->weight, b->weight);
}

/**
 * Initializes the Huffman encoder to encode a given buffer.
 *
 * @param s         Muxing context.
 * @param buffer    Buffer that is to be Huffman-coded.
 * @param size      Size of the buffer.
 * @param sentinel  Set to non-zero if a sentinel should be appended as the last
 *                  symbol in the encoded buffer.
 */
static void pff_huffman_init(AVFormatContext * s,
                             uint8_t const * buffer,
                             size_t size,
                             int sentinel)
{
    PFFMuxContext * context = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != buffer);

    context = (PFFMuxContext *)(s->priv_data);

    memset(context->nodes, 0, sizeof(context->nodes));

    // Create a leaf node for each symbol in the buffer
    for (size_t i = 0; i < size; ++i)
    {
        context->nodes[buffer[i]].value = buffer[i];
        ++context->nodes[buffer[i]].weight;
    }

    // Create a leaf node for the end-of-stream sentinel
    if (sentinel)
    {
        context->nodes[HUFFMAN_SENTINEL_SYMBOL].value = HUFFMAN_SENTINEL_SYMBOL;
        context->nodes[HUFFMAN_SENTINEL_SYMBOL].weight = 1;
    }

    // Sort nodes in descending order by occurrences
    qsort(context->nodes,
          ARRAY_SIZE(context->nodes),
          sizeof(context->nodes[0]),
          &pff_huffman_cmp_node_weight_desc);

    // Count the number of symbols actually present in the buffer
    context->num_symbols = ARRAY_SIZE(context->nodes);
    for (size_t i = 0; i < ARRAY_SIZE(context->nodes); ++i)
    {
        if (0 == context->nodes[i].weight)
        {
            context->num_symbols = i;
            break;
        }
    }
    av_assert0(ARRAY_SIZE(context->nodes) != context->num_symbols);
}

/**
 * Maintains the min-heap property for a heap of HuffmanNode objects.
 *
 * @param heap          The heap.
 * @param heap_length   The number of elements in the heap.
 * @param index         Index of the object to apply the algorithm to.
 *
 * @remark See Introduction to Algorithms, 3rd Edition by Cormen et al., Chapter 6
 */
static void pff_huffman_min_heapify(HuffmanNode ** heap, size_t heap_length, size_t index)
{
    size_t          left        = 0;
    size_t          right       = 0;
    size_t          smallest    = 0;
    HuffmanNode *   temp        = NULL;

    av_assert0(NULL != heap);
    av_assert0(index < heap_length);

    left = 2 * index + 1;
    right = 2 * index + 2;

    if (left < heap_length && heap[left]->weight < heap[index]->weight)
    {
        smallest = left;
    }
    else
    {
        smallest = index;
    }

    if (right < heap_length && heap[right]->weight < heap[smallest]->weight)
    {
        smallest = right;
    }

    if (smallest != index)
    {
        temp = heap[index];
        heap[index] = heap[smallest];
        heap[smallest] = temp;
        // I sure hope this doesn't smash the stack :)
        pff_huffman_min_heapify(heap, heap_length, smallest);
    }
}

/**
 * Pops an element from a min-heap of HuffmanNode objects.
 *
 * @param heap          Heap to pop from.
 * @param heap_length   Number of elements in the heap. Will be updated on return.
 *
 * @return NULL if the heap is empty, or a pointer to the popped element.
 *
 * @remark See Introduction to Algorithms, 3rd Edition by Cormen et al., Chapter 6
 */
static HuffmanNode * pff_huffman_heap_pop(HuffmanNode ** heap, size_t * heap_length)
{
    HuffmanNode *   min = NULL;

    av_assert0(NULL != heap);
    av_assert0(NULL != heap_length);

    if (0 == *heap_length)
    {
        min = NULL;
        goto cleanup;
    }

    min = heap[0];
    heap[0] = heap[*heap_length - 1];
    --(*heap_length);
    if (0 != *heap_length)
    {
        pff_huffman_min_heapify(heap, *heap_length, 0);
    }

cleanup:
    return min;
}

/**
 * Pushes an element to a min-heap of HuffmanNode objects.
 *
 * @param heap              Heap to push onto.
 * @param heap_length       Number of elements in the heap. Will be updated on return.
 * @param heap_max_length   Maximum number of elements allowed on the heap.
 * @param node              Object to push.
 *
 * @remark See Introduction to Algorithms, 3rd Edition by Cormen et al., Chapter 6
 */
static void pff_huffman_heap_push(HuffmanNode ** heap,
                                  size_t * heap_length,
                                  size_t heap_max_length,
                                  HuffmanNode * node)
{
    size_t          index   = 0;
    HuffmanNode *   temp    = NULL;

    av_assert0(NULL != heap);
    av_assert0(NULL != heap_length);
    av_assert0(NULL != node);
    av_assert0(*heap_length < heap_max_length);

    heap[*heap_length] = node;
    ++(*heap_length);

    index = *heap_length - 1;
    while (index > 0 && heap[(index - 1) / 2]->weight > heap[index]->weight)
    {
        temp = heap[index];
        heap[index] = heap[(index - 1) / 2];
        heap[(index - 1) / 2] = temp;

        index = (index - 1) / 2;
    }
}

/**
 * Constructs a Huffman tree.
 *
 * @param s Muxing context.
 *
 * @remark pff_huffman_init must be called prior to this function.
 */
static void pff_huffman_build_tree(AVFormatContext * s)
{
    PFFMuxContext * context                     = NULL;
    HuffmanNode *   queue[MAX_HUFFMAN_SYMBOLS]  = { 0 };
    size_t          queue_length                = 0;
    size_t          next_node_index             = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);

    context = (PFFMuxContext *)(s->priv_data);

    av_assert0(context->num_symbols >= 1);

    //
    // Create a priority queue, and initialize it with the leaf nodes
    //

    for (size_t i = 0; i < context->num_symbols; ++i)
    {
        queue[context->num_symbols - i - 1] = &context->nodes[i];
    }

    queue_length = context->num_symbols;

    // At this point queue is a min-heap

    next_node_index = context->num_symbols;

    while (queue_length > 1)
    {
        // Pop the two nodes with the lowest probability (weight), combine
        // into a single node, and push it back.

        context->nodes[next_node_index].left = pff_huffman_heap_pop(queue, &queue_length);
        context->nodes[next_node_index].right = pff_huffman_heap_pop(queue, &queue_length);
        context->nodes[next_node_index].weight =
            context->nodes[next_node_index].left->weight +
            context->nodes[next_node_index].right->weight;

        context->nodes[next_node_index].left->parent = &context->nodes[next_node_index];
        context->nodes[next_node_index].right->parent = &context->nodes[next_node_index];

        pff_huffman_heap_push(queue,
                              &queue_length,
                              ARRAY_SIZE(queue),
                              &context->nodes[next_node_index]);

        ++next_node_index;
    }

    context->root = pff_huffman_heap_pop(queue, &queue_length);
}

/**
 * Obtains the actual Huffman code from a given Huffman tree *leaf*.
 *
 * @param node  Leaf to obtain the code from.
 * @param code  Code structure to fill.
 *
 * @remark pff_huffman_build_tree must be called prior to this function.
 */
static void pff_huffman_get_code(HuffmanNode * node, HuffmanCode * code)
{
    HuffmanNode *   current         = NULL;
    PutBitContext   put_bit_ctx     = { 0 };
    size_t          written_bytes   = 0;

    av_assert0(NULL != node);
    av_assert0(NULL == node->left);
    av_assert0(NULL == node->right);
    av_assert0(NULL != code);

    code->value = node->value;

    init_put_bits(&put_bit_ctx, code->code, sizeof(code->code) - AV_INPUT_BUFFER_PADDING_SIZE);

    for (current = node; NULL != current->parent; current = current->parent)
    {
        put_bits_le(&put_bit_ctx, 1, (current->parent->left == current) ? 0 : 1);
    }

    code->bit_length = put_bits_count(&put_bit_ctx);

    // We're going to be reading the bits as big endian
    // (because we want them reversed), so pad with zeroes
    // *from the right* until the next byte boundary, flush
    // and reverse the byte order.

    align_put_bits(&put_bit_ctx);

    flush_put_bits_le(&put_bit_ctx);

    written_bytes = put_bytes_output(&put_bit_ctx);

    pff_reverse_byte_array(code->code, written_bytes);
}

/**
 * Comparison function for HuffmanCode structs, comparing by value (symbol)
 * in ascending order.
 */
static int pff_huffman_cmp_code_value_asc(void const * va, void const * vb)
{
    HuffmanCode const * a   = va;
    HuffmanCode const * b   = vb;

    av_assert0(NULL != va);
    av_assert0(NULL != vb);

    return FFDIFFSIGN(a->value, b->value);
}

/**
 * Obtains Huffman codes for all leaves in the tree.
 *
 * The codes are placed in the PFFMuxContext::codes array, sorted by symbol value.
 *
 * @param s Muxing context.
 *
 * @remark pff_huffman_build_tree must be called prior to this function.
 */
static void pff_huffman_get_codes_from_tree(AVFormatContext * s)
{
    PFFMuxContext * context         = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);

    context = (PFFMuxContext *)(s->priv_data);

    av_assert0(context->num_symbols > 1);

    for (size_t i = 0; i < context->num_symbols; ++i)
    {
        pff_huffman_get_code(&context->nodes[i], &context->codes[i]);
    }

    qsort(context->codes,
          context->num_symbols,
          sizeof(context->codes[0]),
          &pff_huffman_cmp_code_value_asc);
}

/**
 * Finds the Huffman code for a given symbol.
 *
 * @param s         Muxing context.
 * @param symbol    Symbol to find a code for.
 *
 * @return  NULL if the symbol doesn't have a matching code, or a pointer to the code
 *          structure.
 *
 * @remark pff_huffman_get_codes_from_tree must be called prior to this function.
 */
static HuffmanCode * pff_huffman_find_code(AVFormatContext * s, uint16_t symbol)
{
    PFFMuxContext * context = NULL;
    HuffmanCode     needle  = { .value = symbol };

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);

    context = (PFFMuxContext *)(s->priv_data);

    return (HuffmanCode *)bsearch(&needle,
                                  context->codes,
                                  context->num_symbols,
                                  sizeof(context->codes[0]),
                                  &pff_huffman_cmp_code_value_asc);
}

/**
 * Serializes the current Huffman tree to the PFF format.
 *
 * @param s                 Muxing context with an already-built Huffman tree.
 * @param serialized_tree   Will receive the serialized tree. The buffer must
 *                          have at least 2*l elements, with l being the number of
 *                          leaves in the tree.
 */
static void pff_huffman_serialize_tree(AVFormatContext * s, uint16_t * serialized_tree)
{
    PFFMuxContext * context                                 = NULL;
    HuffmanNode *   to_visit[2 * MAX_HUFFMAN_SYMBOLS - 1]   = { 0 };
    size_t          to_visit_length                         = 0;
    uint16_t        output_index                            = 0;
    HuffmanNode *   current                                 = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != serialized_tree);

    context = (PFFMuxContext *)(s->priv_data);

    serialized_tree[0] = 2 * context->num_symbols - 1;

    to_visit[0] = context->root;
    to_visit_length = 1;

    output_index = 1;

    while (to_visit_length > 0)
    {
        // Pop a node from the stack
        current = to_visit[to_visit_length - 1];
        --to_visit_length;

        if (NULL == current->left || NULL == current->right)
        {
            // Leaf node. Write the symbol into the current slot.

            av_assert0(current->left == current->right);

            serialized_tree[output_index] = 0x8000 | current->value;
        }
        else
        {
            // Internal node.

            av_assert0(NULL != current->right);
            av_assert0(NULL != current->left);

            // Schedule the children to be visited.
            to_visit[to_visit_length++] = current->right;
            to_visit[to_visit_length++] = current->left;

            // Set the current output index as the parent index in the children.
            // When they are visited, they'll use this index to set the correct
            // offset.
            current->right->parent_index = current->left->parent_index = output_index;
        }

        // If we have a parent, update it to point to our index.
        // Note that this will happen twice: first for the left child, then for
        // the right. This is fine, since the left child is always placed immediately
        // after the parent.
        if (NULL != current->parent)
        {
            // Bias the index by -1 since we started at index 1
            serialized_tree[current->parent_index] = output_index - 1;
        }

        ++output_index;
    }

    av_assert0(output_index == 2 * context->num_symbols);
}

/**
 * Adds two size_t values, checking for overflow.
 *
 * @return 0 on success, AVERROR(EINVAL) on overflow.
 */
static int pff_safe_add_size_t(size_t a, size_t b, size_t * c)
{
    *c = a + b;
    if (*c < a)
    {
        *c = SIZE_MAX;
        return AVERROR(EINVAL);
    }
    return 0;
}

/**
 * Computes the size of the buffer required to hold the Huffman encoding
 * of a given array of bytes.
 *
 * @param s             Muxing context.
 * @param buffer        Input byte buffer.
 * @param size          Size of the input buffer.
 * @param encoded_size  Will receive the encoded size.
 *
 * @return 0 on success or an AVERROR value on failure.
 *
 * @remark pff_huffman_get_codes_from_tree must be called prior to this function.
 */
static int pff_huffman_encoded_buffer_size(AVFormatContext * s,
                                           uint8_t const * buffer,
                                           size_t size,
                                           size_t * encoded_size)
{
    int             result              = AVERROR_BUG;
    HuffmanCode *   code                = NULL;
    size_t          encoded_bit_count   = 0;

    if (NULL == buffer || 0 == size)
    {
        return 0;
    }

    for (size_t i = 0; i < size; ++i)
    {
        code = pff_huffman_find_code(s, buffer[i]);
        av_assert0(NULL != code);

        result = pff_safe_add_size_t(encoded_bit_count, code->bit_length, &encoded_bit_count);
        if (result < 0)
        {
            av_log(s, AV_LOG_FATAL, "Integer overflow in Huffman encoding\n");
            goto cleanup;
        }
    }

    code = pff_huffman_find_code(s, HUFFMAN_SENTINEL_SYMBOL);
    if (NULL != code)
    {
        result = pff_safe_add_size_t(encoded_bit_count, code->bit_length, &encoded_bit_count);
        if (result < 0)
        {
            av_log(s, AV_LOG_FATAL, "Integer overflow in Huffman encoding\n");
            goto cleanup;
        }
    }

    *encoded_size = encoded_bit_count / 8;
    if (0 != encoded_bit_count % 8)
    {
        ++(*encoded_size);
    }

    *encoded_size = FFALIGN(*encoded_size, 4);

    result = 0;

cleanup:
    return result;
}

/**
 * Encodes a buffer using the current Huffman tree.
 *
 * @param s             Muxing context with an initialized Huffman tree.
 * @param buffer        Buffer to encode.
 * @param size          Size of the input buffer.
 * @param output        Output buffer. Must be at least as large as indicated by
 *                      pff_huffman_encoded_buffer_size.
 * @param output_size   Output buffer size.
 */
static void pff_huffman_encode_buffer(AVFormatContext * s,
                                      uint8_t const * buffer,
                                      size_t size,
                                      uint8_t * output,
                                      size_t output_size)
{
    int             result          = AVERROR_BUG;
    HuffmanCode *   code            = NULL;
    PutBitContext   put_bit_ctx     = { 0 };
    GetBitContext   get_bit_ctx     = { 0 };
    size_t          bytes_written   = 0;

    init_put_bits(&put_bit_ctx, output, output_size);

    for (size_t i = 0; i < size; ++i)
    {
        code = pff_huffman_find_code(s, buffer[i]);
        av_assert0(NULL != code);

        result = init_get_bits(&get_bit_ctx, code->code, code->bit_length);
        av_assert0(result >= 0);  // This really shouldn't fail.

        while (get_bits_left(&get_bit_ctx) > 0)
        {
            put_bits(&put_bit_ctx, 1, get_bits1(&get_bit_ctx));
        }
    }

    code = pff_huffman_find_code(s, HUFFMAN_SENTINEL_SYMBOL);
    if (NULL != code)
    {
        result = init_get_bits(&get_bit_ctx, code->code, code->bit_length);
        av_assert0(result >= 0);  // This really shouldn't fail.

        while (get_bits_left(&get_bit_ctx) > 0)
        {
            put_bits(&put_bit_ctx, 1, get_bits1(&get_bit_ctx));
        }
    }

    flush_put_bits(&put_bit_ctx);

    bytes_written = put_bytes_output(&put_bit_ctx);
    bytes_written = FFALIGN(bytes_written, 4);

    // Swap every 4 bytes to match the PFF format
    for (size_t i = 0; i < bytes_written; i += 4)
    {
        AV_WL32(&output[i], AV_RB32(&output[i]));
    }
}

/**
 * Huffman-codes a buffer.
 *
 * @param s                     Muxing context.
 * @param buffer                Buffer to encode.
 * @param size                  Size of the input buffer.
 * @param output_buffer         Will receive the encoded buffer.
 * @param output_buffer_size    Will receive the encoded buffer's size.
 *
 * @return 0 on success or an AVERROR value on failure.
 *
 * @remark Free the returned buffer using av_free.
 */
static int pff_huffman_encode(AVFormatContext * s,
                              uint8_t const * buffer,
                              size_t size,
                              uint8_t ** output_buffer,
                              size_t * output_buffer_size)
{
    int             result                  = AVERROR_BUG;
    PFFMuxContext * context                 = NULL;
    size_t          serialized_tree_size    = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != buffer);
    av_assert0(NULL != output_buffer);

    context = (PFFMuxContext *)(s->priv_data);

    if (size < 1)
    {
        // pff_huffman_get_code will freak out
        av_log(s, AV_LOG_FATAL, "Empty buffers are not supported by Huffman encoder\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    pff_huffman_init(s, buffer, size, 1);

    pff_huffman_build_tree(s);

    pff_huffman_get_codes_from_tree(s);

    result = pff_huffman_encoded_buffer_size(s, buffer, size, output_buffer_size);
    if (result < 0)
    {
        av_log(s, AV_LOG_FATAL, "Couldn't compute Huffman-encoded buffer size\n");
        goto cleanup;
    }

    // Add room for the tree
    serialized_tree_size = 2 * context->num_symbols * sizeof(uint16_t);
    *output_buffer_size += serialized_tree_size;

    *output_buffer = av_mallocz(*output_buffer_size);   // Zero memory to avoid data leaks
    if (NULL == *output_buffer)
    {
        result = AVERROR(ENOMEM);
        goto cleanup;
    }

    pff_huffman_serialize_tree(s, (uint16_t *)(*output_buffer));

    pff_huffman_encode_buffer(s,
                              buffer,
                              size,
                              *output_buffer + serialized_tree_size,
                              *output_buffer_size - serialized_tree_size);

    result = 0;

cleanup:
    if (result < 0)
    {
        av_freep(output_buffer);
    }

    return result;
}

/**
 * Initializes the muxing state.
 */
static int pff_init(AVFormatContext * s)
{
    int             result  = AVERROR_BUG;
    PFFMuxContext * context = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->priv_data);

    context = (PFFMuxContext *)(s->priv_data);
    memset(context, 0, sizeof(*context));

    if (1 != s->nb_streams)
    {
        av_log(s, AV_LOG_FATAL, "Exactly one stream is required\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    if (AV_CODEC_ID_DDS != s->streams[0]->codecpar->codec_id)
    {
        av_log(s, AV_LOG_FATAL, "Only DDS video is supported\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    result = 0;

cleanup:
    return result;
}

/**
 * Writes the PFF header.
 */
static int pff_write_header(AVFormatContext * s)
{
    PFFMuxContext * context = NULL;

    av_assert0(NULL != s);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != s->priv_data);

    context = (PFFMuxContext *)(s->priv_data);

    avio_write(s->pb, PFF_HEADER, sizeof(PFF_HEADER));

    avio_write_marker(s->pb, AV_NOPTS_VALUE, AVIO_DATA_MARKER_FLUSH_POINT);

    context->first_frame = 1;

    return 0;
}

/**
 * Writes the PFF trailer.
 */
static int pff_write_trailer(AVFormatContext * s)
{
    av_assert0(NULL != s);
    av_assert0(NULL != s->pb);

    avio_put_str(s->pb, PFF_TRAILER);

    avio_write_marker(s->pb, AV_NOPTS_VALUE, AVIO_DATA_MARKER_FLUSH_POINT);

    return 0;
}

/**
 * Writes a single packet to the PFF file.
 */
static int pff_write_packet(AVFormatContext * s, AVPacket * pkt)
{
    int             result              = AVERROR_BUG;
    PFFMuxContext * context             = NULL;
    uint8_t const * dds                 = NULL;
    uint8_t const * pixel_data          = NULL;
    size_t          pixel_data_size     = 0;
    uint32_t        video_section_size  = 0;
    uint8_t *       encoded_pixels      = NULL;
    size_t          encoded_pixels_size = 0;
    uint32_t        frame_size          = 0;

    av_assert0(NULL != s);
    av_assert0(NULL != s->pb);
    av_assert0(NULL != s->priv_data);
    av_assert0(NULL != pkt);    // We didn't set AVFMT_ALLOW_FLUSH

    context = (PFFMuxContext *)(s->priv_data);

    av_assert0(1 == s->nb_streams);     // We support only one stream. Checked in pff_init.
    av_assert0(0 == pkt->stream_index);

    if (AV_NOPTS_VALUE == pkt->pts)
    {
        av_log(s, AV_LOG_FATAL, "Missing packet timestamp\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    if (pkt->size < sizeof(context->dds))
    {
        av_log(s, AV_LOG_FATAL, "Packet doesn't contain a DDS header\n");
        result = AVERROR_INVALIDDATA;
        goto cleanup;
    }

    dds = pkt->data;
    pixel_data = &pkt->data[sizeof(context->dds)];
    pixel_data_size = pkt->size - sizeof(context->dds);

    video_section_size = sizeof(uint8_t); // Flags

    if (context->first_frame)
    {
        if (!(AV_RL32(&dds[0x50]) & 0x4))  // !(ddspf.dwFlags & DDPF_FOURCC)
        {
            av_log(s, AV_LOG_FATAL, "Missing DDPF_FOURCC flag in DDS header\n");
            result = AVERROR_PATCHWELCOME;
            goto cleanup;
        }
        if (MKBETAG('D', 'X', 'T', '1') != AV_RB32(&dds[0x54]))  // ddspf.dwFourCC
        {
            av_log(s, AV_LOG_FATAL, "Unsupported FourCC: 0x%08" PRIX32 "\n", AV_RB32(&dds[0x54]));
            result = AVERROR_PATCHWELCOME;
            goto cleanup;
        }

        memmove(context->dds, dds, sizeof(context->dds));

        // https://docs.microsoft.com/en-us/windows/win32/direct3ddds/dds-file-layout-for-textures
        context->decompressed_size =
            FFMAX(1, ((s->streams[0]->codecpar->width + 3) / 4))
            * FFMAX(1, ((s->streams[0]->codecpar->height + 3) / 4))
            * 8;

        video_section_size +=
            sizeof(uint32_t)        // Decompressed size
            + sizeof(context->dds)
        ;
    }
    else
    {
        if (0 != memcmp(context->dds, dds, sizeof(context->dds)))
        {
            av_log(s, AV_LOG_FATAL, "DDS header mismatch");
            result = AVERROR_INVALIDDATA;
            goto cleanup;
        }
    }

    result = pff_huffman_encode(s,
                                pixel_data, pixel_data_size,
                                &encoded_pixels, &encoded_pixels_size);
    if (result < 0)
    {
        goto cleanup;
    }

    if (encoded_pixels_size > INT_MAX)
    {
        av_log(s, AV_LOG_FATAL, "Encoded frame is too large\n");
        result = AVERROR_PATCHWELCOME;
        goto cleanup;
    }

    video_section_size += encoded_pixels_size;

    frame_size =
        + sizeof(uint64_t)              // Timestamp, a double
        + sizeof(VIDEO_SECTION_MAGIC)
        + sizeof(uint32_t)              // Video section size
        + video_section_size
        + sizeof(END_FRAME_MAGIC)
    ;

    avio_put_str(s->pb, FRAME_MAGIC);
    avio_wl32(s->pb, frame_size);
    avio_wl64(s->pb, ((av_alias64){.f64 = pkt->pts * av_q2d(s->streams[0]->time_base)}).u64);

    avio_put_str(s->pb, VIDEO_SECTION_MAGIC);
    avio_wl32(s->pb, video_section_size);
    if (context->first_frame)
    {
        avio_wl32(s->pb, context->decompressed_size);
        avio_write(s->pb, context->dds, sizeof(context->dds));
    }
    avio_w8(s->pb, VIDEO_FLAG_COMPRESSED);
    avio_write(s->pb, encoded_pixels, (int)encoded_pixels_size);

    avio_put_str(s->pb, END_FRAME_MAGIC);

    avio_write_marker(s->pb, AV_NOPTS_VALUE, AVIO_DATA_MARKER_FLUSH_POINT);

    context->first_frame = 0;

    result = 0;

cleanup:
    av_freep(&encoded_pixels);

    return result;
}


const AVOutputFormat ff_pff_muxer = {
    .name           = "pff",
    .long_name      = NULL_IF_CONFIG_SMALL("Still Life 2 PFF"),
    .extensions     = "pff",
    .video_codec    = AV_CODEC_ID_DDS,
    .audio_codec    = AV_CODEC_ID_VORBIS,

    .init           = &pff_init,
    .write_header   = &pff_write_header,
    .write_trailer  = &pff_write_trailer,
    .write_packet   = &pff_write_packet,
    .priv_data_size = sizeof(PFFMuxContext),
};
