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
 * @defgroup    SWIF-codec    (former) Linear Coding Library
 * @ingroup     swif-codec
 * @brief       (former lib. for) linear coding and decoding of packets.
 * @{
 *
 * @file
 * @brief   Algebraic operations on vectors (sequence) of elements of
 *          the finite fields GF(2), GF(4), GF(16) or GF(256):
 *          basically addition, linear combinations.
 *
 * @author  Cedric Adjih <cedric.adjih@inria.fr>
 */

#ifndef __LINEAR_CODE_H__
#define __LINEAR_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/

#include <stdint.h>

/*---------------------------------------------------------------------------*/

/**
 * @brief The "log2 of the number of bits (per element)", also denoted `L`,
 *        is used everywhere
 *        to specify which of GF(2), GF(4), GF(16), GF(256) is used
 *        (`L` is `0,1,2,3` respectively).
 *        The maximum (included) is 3 for GF(256).
 * @details Notice: \f$2^{(2^0)} = 2\f$, \f$2^{(2^1)} = 4\f$,
 *          \f$2^{(2^2)} = 16\f$, \f$2^{(2^3)} = 256\f$
 */
#define MAX_LOG2_NB_BIT_COEF 3 /* included */

/**
 * @brief Returns the inverse of an element
 *        in GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in] x - must be different from `0`
 * @return    y in the finite field such that `x.y = 1`
 * @details An `assert` verifies that `x != 0`
 *        (if assertions are disabled, `0` or `1` are expected to be returned).
 */
uint8_t lc_inv(uint8_t x, uint8_t log2_nb_bit_coef);

/**
 * @brief Returns the product of elements
 *        in GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in] x is one element of the finite field
 * @param[in] y is one element of the finite field
 * @return    x.y in the galois field (0 <= x.y < \f$2^{(2^L)}\f$)
 */
uint8_t lc_mul(uint8_t x, uint8_t y, uint8_t log2_nb_bit_coef);

/**
 * @brief Returns the opposite of some value
 *        in GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in] x is one element of the finite field
 * @return    -x in the galois field (0 <= `-x` < \f$2^{(2^L)}\f$)
 * @details in characteristic 2 (e.g. GF(\f$2^n\f$), `-x == x`
 */
static inline uint8_t lc_neg(uint8_t x, uint8_t log2_nb_bit_coef)
{ (void)log2_nb_bit_coef; return x; /* -x == x in GF(2^n) */ }

/*---------------------------------------------------------------------------*/

/**
 * @brief Divide a value by a power of 2
 * @param[in] value
 * @param[in] log2_divisor
 * @return    value / \f$2^{log2\_divisor}\f$
 */
#define DIV_LOG2(value, log2_divisor) ((value)>>(log2_divisor))

/**
 * @brief Multiply a value by a power of 2
 * @param[in] value
 * @param[in] log2_divisor
 * @return    value / \f$2^{log2\_divisor}\f$
 */
#define MUL_LOG2(value, log2_mult) ((value)<<(log2_mult))

/**
 * @brief Return a mask
 * @param[in] nb_bit   Number of bits in the mask
 * @return    11111111..11 with `nb_bit` `1`
 * @details   it is an `int`, check overflow for larger `nb_bits`
 */
#define MASK(nb_bit) ((1 << (nb_bit))-1)

/**
 * @brief Compute the remainder of value divided by a power of 2
 * @param[in] value
 * @param[in] log2_divisor
 * @return    value `modulo` \f$2^{log2\_divisor}\f$
 */
#define MOD_LOG2(value, log2_divisor) ((value)&MASK((log2_divisor)))


/**
 * \f$2^3\f$ = 8 bits per byte.
 */
#define LOG2_BITS_PER_BYTE 3

/**
 * @brief Compute the sum of two vectors (sequences, arrays) of elements
 *         of a finite field
 * @param[in]  data1       First vector of elements
 * @param[in]  size1       Number of bytes (not elements) in the first vector.
 * @param[in]  data2       Second vector of elements
 * @param[in]  size2       Number of bytes (not elements) in the first vector.
 * @param[out] result      Output vector holding the sum of the elements,
 *                         e.g. `result[i] = data1[i]+data2[i]`
 *                         in the finite field; the shortest of sequence
 *                         is considered to by extended with `0`.
 *                         Called must ensure that the memory area of `result`
 *                         has at least `result_size` bytes.
 * @param[out] result_size Maximum of `size1` and `size2`.
 * @details `data1`, `data2` and `result` could be equal as pointers (two of
 *          them or all three), and the `result` would still be correct.
 *          Other than such strict equality, `result` should point to memory
 *          areas disjoint from `data1` or `data2`,
 *          e.g. `lc_vector_add(p,l-1,p,l-1,p+1,&l2)` gives improper result.
 */
void lc_vector_add(uint8_t* data1, uint16_t size1,
		   uint8_t* data2, uint16_t size2,
		   uint8_t* result, uint16_t* result_size);

/**
 * @brief Multiply each element of one vector (sequences, arrays) of elements
 *        by one given element of the finite field.
 * @param[in]  coef       Coefficient of the finite field
 * @param[in]  data       Vector of elements
 * @param[in]  size       Number of bytes (not elements) in the vector
 * @param[in]  log2_nb_bit_coef Defines the finite field, e.g.
 *                        GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[out] result      Output vector holding the sum of the elements,
 *                         e.g. `result[i] = coef x data[i]`
 *                         in the finite field;
 * @details `data` and `result` could be equal as pointers,
 *          and the `result` would still be correct.
 *          Other than such strict equality, `result` should point to memory
 *          area disjoint from `data`,
 *          e.g. `lc_vector_mul(c,p,l-1,L,p+1)` gives improper result.
 */
void lc_vector_mul(uint8_t coef, uint8_t* data, uint16_t size,
		   uint8_t log2_nb_bit_coef, uint8_t* result);

/**
 * @brief Set the n-th element of one vector (sequences, arrays)
 *         to one given element of one finite field.
 * @param[in]  data       Vector of elements
 * @param[in]  size       Number of bytes (not elements) in the vector
 * @param[in]  log2_nb_bit_coef Defines the finite field, e.g.
 *                        GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in]  coef_pos   Position of the element to set in the vector
 *                        (as \f$n-th\f$ element, not position in bytes).
 * @param[in]  coef_value The value of the finite field to put in the vector
 *                        at the position `coef_pos`
 * @details    assertions verify the lack of `out of bound` access
 */
void lc_vector_set(uint8_t* data, uint16_t size, uint8_t log2_nb_bit_coef,
		   uint16_t coef_pos, uint8_t coef_value);

/**
 * @brief Get the n-th element of one vector (sequences, arrays)
 *        of elements of one finite field.
 * @param[in]  data       Vector of elements
 * @param[in]  size       Number of bytes (not elements) in the vector
 * @param[in]  log2_nb_bit_coef Defines the finite field, e.g.
 *                        GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in]  coef_pos   Position of the element to set in the vector
 *                        (as \f$n-th\f$ element, not position in bytes).
 * @return     The value in the vector at the position `coef_pos` (e.g.
 *             a finite field element)
 * @details    assertions verify the lack of `out of bound` access
 */
uint8_t lc_vector_get(uint8_t* data, uint16_t size, uint8_t log2_nb_bit_coef,
		      uint16_t coef_pos);

  /**
 * @brief Convert the position of n-th element of one vector (sequences, arrays)
 *        of one finite field to the position in expressed in bytes (of memory).
 * @param[in]  log2_nb_bit_coef Defines the finite field, e.g.
 *                        GF(\f$2^{(2^L)}\f$) where `L = log2_nb_bit_coef`
 * @param[in]  coef_pos   Position of the element to set in the vector
 *                        (as \f$n-th\f$ element, not position in bytes).
 * @return     The value in the vector at the position `coef_pos` (e.g.
 *             a finite field element)
 * @details    In GF(256), position (index) `coef_pos` in vector is same 
 *             as position in bytes;
 *             in GF(2), position in bytes is `coef_pos/8`, etc.
 */
uint16_t lc_vector_get_byte_pos_of_coef(uint8_t log2_nb_bit_coef,
					uint16_t coef_pos);

/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* __LINEAR_CODE_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
