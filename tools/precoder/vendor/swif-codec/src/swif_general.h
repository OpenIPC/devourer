/*---------------------------------------------------------------------------
 * Copyright 2013-2017 Inria
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *---------------------------------------------------------------------------*/

/**
 * @defgroup    SWiF-Codec
 * @ingroup     swif-codec-base
 * @brief       an open-source sliding window FEC codec in C
 * @{
 *
 * @file
 * @brief   General definitions and macros
 *
 * @author  Cedric Adjih <cedric.adjih@inria.fr>
 * @author  Vincent Roca
 */

#ifndef __SWIF_GENERAL_H__
#define __SWIF_GENERAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/

#include <stdint.h>

/*---------------------------------------------------------------------------*/

#define BEGIN_MACRO do {
#define END_MACRO } while(0)

/*---------------------------------------------------------------------------*/

#define BITS_PER_BYTE 8
#define LOG2_BITS_PER_BYTE 3

#define BOOL_FALSE 0
#define BOOL_TRUE  1

#define REQUIRE(...) ASSERT(__VA_ARGS__)

#define STATIC_ENSURE_EQUAL(name,a,b) \
  struct name { char l1[(a)-(b)]; char l2[(b)-(a)]; };

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

static inline uint16_t min_except(uint16_t v1, uint16_t v2, uint16_t ignored)
{
  if (v1 == ignored) 
    return v2;
  if (v2 == ignored)
    return v1;
  return MIN(v1, v2);
}

static inline uint16_t max_except(uint16_t v1, uint16_t v2, uint16_t ignored)
{
  if (v1 == ignored) 
    return v2;
  if (v2 == ignored)
    return v1;
  return MAX(v1, v2);
}

/*---------------------------------------------------------------------------*/

#ifdef DEBUG
#define DEBUG_PRINT(...) do { fprintf( stderr, __VA_ARGS__ ); } while(0)
#else
#define DEBUG_PRINT(...) do {} while(0)
#endif

static inline void DEBUG_DUMP(void *data, unsigned int data_size)
{
        for (unsigned int j=0; j<data_size; j++)
                DEBUG_PRINT(" %02x", ((uint8_t *)(data))[j]);
}

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __SWIF_GENERAL_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
