/**
 * SWiF Codec: an open-source sliding window FEC codec in C
 * https://github.com/irtf-nwcrg/swif-codec
 */

/*---------------------------------------------------------------------------*/

#include <assert.h>
#include "swif_symbol.h"

/*---------------------------------------------------------------------------*/

#include "swif_table-mul-gf256.c"

/* gf256_add and gf256_sub are inline */

uint8_t gf256_mul(uint8_t a, uint8_t b)
{
    return gf256_mul_table[a][b];
}

uint8_t gf256_inv(uint8_t a)
{
    return gf256_inv_table[a]; 
}

uint8_t gf256_div(uint8_t a, uint8_t b)
{
    return gf256_mul(a, gf256_inv(b));
}

/*---------------------------------------------------------------------------*/

/**
 * @brief Take a symbol and add another symbol multiplied by a 
 *        coefficient, e.g. performs the equivalent of: p1 += coef * p2
 * @param[in]  symbol1  First symbol (to which coef*symbol2 will be added)
 * @param[in]  coef     Coefficient by which the second packet is multiplied
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
void symbol_add_scaled
(void *symbol1, uint8_t coef, void *symbol2, uint32_t symbol_size)
{
    uint8_t *data1 = (uint8_t *) symbol1;
    uint8_t *data2 = (uint8_t *) symbol2;
    for (uint32_t i=0; i<symbol_size; i++) {
        data1[i] ^= gf256_mul(coef, data2[i]);
    }
}

/**
 * @brief Take a symbol and add another symbol to it,
 *        e.g. performs the equivalent of: result = symbol1 + symbol2
 * @param[in]  symbol1  First symbol (to which symbol2 will be added)
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
void symbol_add
(void *symbol1, void *symbol2, uint32_t symbol_size, uint8_t* result)
{
    uint8_t *data1 = (uint8_t *) symbol1;
    uint8_t *data2 = (uint8_t *) symbol2;
    for (uint32_t i=0; i<symbol_size; i++) {
        result[i] = gf256_add(data1[i], data2[i]);
    }
}

/**
 * @brief Take a symbol and subtract another symbol to it,
 *        e.g. performs the equivalent of: result = p1 - p2
 * @param[in]  symbol1  First symbol (to which symbol2 will be subtracted)
 * @param[in]  symbol2  Second symbol
 * @param[out] result   Symbol where the result is stored
 */
/* Same as addidtion for GF(2^8) */
void symbol_sub
(void *symbol1, void *symbol2, uint32_t symbol_size, uint8_t* result)
{
    symbol_add(symbol1, symbol2, symbol_size, result);
}

/**
 * @brief Take a symbol and multiply it by a coefficient,
 *        e.g. performs the equivalent of: result = coef * p1 
 * @param[in]  symbol1  Initial symbol
 * @param[in]  coeff    Coefficient by which the symbol is multiplied (scaled)
 * @param[out] result   Symbol where the result is stored
 */
void symbol_mul
(void *symbol1, uint8_t coeff, uint32_t symbol_size, uint8_t* result)
{
    if (coeff == 0) {
	memset(result, 0, symbol_size);
	return;
    }
    uint8_t *data = (uint8_t *) symbol1;
    for (uint32_t i=0; i<symbol_size; i++) {
        result[i] = gf256_mul(coeff, data[i]);
    }
}

/**
 * @brief Take a symbol and divide it by a coefficient, 
 *         e.g. performs the equivalent of: result = (1/coeff) * p1
 * @param[in]  symbol1  Initial symbol
 * @param[in]  coeff    Coefficient by which the symbol is divided
 * @param[out] result   Symbol where the result is stored
 */
void symbol_div
(void *symbol1,  uint32_t symbol_size, uint8_t coeff, uint8_t* result)
{
    assert(coeff != 0);
    uint8_t *data = (uint8_t *) symbol1;
    for (uint32_t i=0; i<symbol_size; i++) {
        result[i] = gf256_div(data[i], coeff);
    }
}
/*---------------------------------------------------------------------------*/
