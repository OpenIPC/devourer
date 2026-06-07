/**
 * @defgroup    SWiF-Codec
 * @ingroup     swif-codec-linear
 * @brief       an open-source sliding window FEC codec in C (linear operations)
 * @{
 *
 * @file
 * @brief       A symbol is a vector of GF(256) elements
 *
 * @author      Oumaima Attia <oumaima.attia@inria.fr>
 * @author      Cedric Adjih <cedric.adjih@inria.fr>
 */

/**
 * SWiF Codec: 
 * https://github.com/irtf-nwcrg/swif-codec
 */

#ifndef __SWIF_SYMBOL_H__
#define __SWIF_SYMBOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
  
/*---------------------------------------------------------------------------*/
/* 
 * Operations in GF(2^8)
 */
  
/* Add two numbers in a GF(2^8) finite field: a+b 
 * GF(2^8) is of characteristic 2, so 1 + 1 = 0, and addition, 
 * subtraction and exclusive-or are all the same. */
static inline uint8_t gf256_add(uint8_t a, uint8_t b)
{ return a^b; }

/* Subtract two numbers in a GF(2^8) finite field: a-b */
static inline uint8_t gf256_sub(uint8_t a, uint8_t b)
{ return gf256_add(a,b); }

/* Get the inverse of an element in a GF(2^8) finite field: 1/a
   note: returns 0 for element 0 (instead of an exception) */
uint8_t gf256_inv(uint8_t a);

/* Divide two elements in the finite field GF(2^8): a/b 
   note: returns 0 if `b' is 0 (instead of an exception) */
uint8_t gf256_div(uint8_t a, uint8_t b);

/* Multiply two elements in the finite field GF(2^8): a*b 
   note: returns 0 if `b' is 0 (instead of an exception) */
uint8_t gf256_mul(uint8_t a, uint8_t b);
  
/*---------------------------------------------------------------------------*/

/* invariant: the functions below work with NULL pointers to symbol
   iff symbol_size is 0 (as they will not actually dereference the pointer) */

/**
 * @brief Take a symbol and add another symbol multiplied by a 
 *        coefficient, e.g. performs the equivalent of: p1 += coef * p2
 * @param[in]  symbol1  First symbol (to which coef*symbol2 will be added)
 * @param[in]  coef     Coefficient by which the second packet is multiplied
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
void symbol_add_scaled
(void *symbol1, uint8_t coef, void *symbol2, uint32_t symbol_size);


/**
 * @brief Take a symbol and add another symbol to it,
 *        e.g. performs the equivalent of: result = symbol1 + symbol2
 * @param[in]  symbol1  First symbol (to which symbol2 will be added)
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
void symbol_add
(void *symbol1, void *symbol2, uint32_t symbol_size, uint8_t* result);


/**
 * @brief Take a symbol and subtract another symbol to it,
 *        e.g. performs the equivalent of: result = p1 - p2
 * @param[in]  symbol1  First symbol (to which symbol2 will be subtracted)
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
void symbol_sub
(void *symbol1, void *symbol2, uint32_t symbol_size, uint8_t* result);


/**
 * @brief Take a symbol and multiply it by a coefficient,
 *        e.g. performs the equivalent of: result = coef * p1 
 * @param[in]  symbol1  Initial symbol
 * @param[in]  coeff    Coefficient by which the symbol is multiplied (scaled)
 * @param[out] result   Symbol where the result is stored
 */
void symbol_mul
(void *symbol1, uint8_t coeff, uint32_t symbol_size, uint8_t* result);


/**
 * @brief Take a symbol and divide it by a coefficient, 
 *         e.g. performs the equivalent of: result = (1/coeff) * p1
 * @param[in]  symbol1  Initial symbol
 * @param[in]  coeff    Coefficient by which the symbol is divided
 * @param[out] result   Symbol where the result is stored
 * Note: there is an assertion `assert(coeff != 0)'
 */
void symbol_div
(void *symbol1, uint32_t symbol_size, uint8_t coeff, uint8_t* result);


/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __SWIF_SYMBOL_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
