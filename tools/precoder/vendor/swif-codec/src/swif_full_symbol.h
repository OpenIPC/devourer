/**
 * @defgroup    SWiF-Codec
 * @ingroup     swif-codec-linear
 * @brief       an open-source sliding window FEC codec in C (linear operations)
 * @{
 *
 * @file
 * @brief       A "full symbol" is a symbol with a full specification of
 *              what it represents, e.g. an encoding header.
 *
 * @author      Oumaima Attia <oumaima.attia@inria.fr>
 */

/**
 * SWiF Codec: 
 * https://github.com/irtf-nwcrg/swif-codec
 */

#ifndef __SWIF_FULL_SYMBOL_H__
#define __SWIF_FULL_SYMBOL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* There were initially two designed implementations of the full_symbol
 * - one reimplemented in swif_full_symbol_impl.c (DEFAULT ONE)
 * - one as wrapper on top of liblc (NO LONGER USED, fixed size, no malloc)
 */
  
/*---------------------------------------------------------------------------*/

/**
 * A symbol (e.g. a packet, coded of not, seen as an array of GF(xx) coefs.
 */
typedef struct s_swif_full_symbol_t swif_full_symbol_t;

/**
 * A set of symbols. You can add symbols, remove symbols, etc.
 */
typedef struct s_swif_full_symbol_set_t swif_full_symbol_set_t;

/**
 * Used for a non-specified/non-existing value of symbol_id
 */
#define SYMBOL_ID_NONE 0xfffffffful

/**
 * An identifier for a source symbol (source symbol index)
 */
typedef uint32_t symbol_id_t;
    
/**
 * @brief Create a full_symbol from a source symbol
 */
swif_full_symbol_t *full_symbol_create_from_source
(uint32_t symbol_id, uint8_t *symbol_data, uint32_t symbol_size);

/**
 * @brief Create a full_symbol from a set of coeffiicents and a symbol
 *        typically this is provided from a received repair packet
 */
swif_full_symbol_t *full_symbol_create
(uint8_t *symbol_coef_table, uint32_t min_symbol_id, uint32_t nb_symbol_id,
 uint8_t *symbol_data, uint32_t symbol_size);

/**
 * @brief Get the size of the data
 */
uint32_t full_symbol_get_size(swif_full_symbol_t *full_symbol);

/**
 * @brief Get the minimum source index that appears in the symbol
 *        returns SYMBOL_ID_NONE if there is none (e.g. symbol is 0)
 */
uint32_t full_symbol_get_min_symbol_id(swif_full_symbol_t *full_symbol);

/**
 * @brief Get the maximum source index that appears in the symbol
 *        returns SYMBOL_ID_NONE if there is none (e.g. symbol is 0)
 */
uint32_t full_symbol_get_max_symbol_id(swif_full_symbol_t *full_symbol);

/**
 * @brief Get the symbol 'data'. result_data should be a pointer to a
 *        a block of memory of size at least full_symbol_get_size(full_symbol)
 */
void full_symbol_get_data
(swif_full_symbol_t *full_symbol, uint8_t *result_data);


/**
 * @brief Get the coefficient corresponding to the specified symbol identifier
 */
uint8_t full_symbol_get_coef
(swif_full_symbol_t *full_symbol, uint32_t symbol_id);

/**
 * @brief Return the number of coefficients present in the symbol
 *        (max symbol id - min symbol id)+1;
 *        Note: returns zero if there are none (e.g. symbol is zero)
 */
static inline uint32_t full_symbol_count_coef(swif_full_symbol_t *full_symbol)
{
    uint32_t min_symbol_id = full_symbol_get_min_symbol_id(full_symbol); 
    uint32_t max_symbol_id = full_symbol_get_max_symbol_id(full_symbol);

    if (min_symbol_id == SYMBOL_ID_NONE) {
        return 0;
    } else {
        return max_symbol_id-min_symbol_id+1;
    }
}

/**
 * @brief Returns whether the symbol is an empty symbol
 */
static inline bool full_symbol_is_zero(swif_full_symbol_t *full_symbol)
{
    return full_symbol_get_min_symbol_id(full_symbol) == SYMBOL_ID_NONE;
}

/**
 * @brief Returns whether the symbol is an empty symbol
 */
static inline bool full_symbol_has_one_id(swif_full_symbol_t *full_symbol)
{
  return (!full_symbol_is_zero(full_symbol))
    && (full_symbol_get_min_symbol_id(full_symbol)
	== full_symbol_get_max_symbol_id(full_symbol));
}

 
/**
 * @brief Release a full_symbol
 */
void full_symbol_free(swif_full_symbol_t *swif_full_symbol);

/**
 * @brief Create a new (unlinked) copy of a full_symbol
 */
swif_full_symbol_t *full_symbol_clone(swif_full_symbol_t *swif_full_symbol);

/**
 * @brief Dump a full symbol on a FILE
 */    
void full_symbol_dump(swif_full_symbol_t *full_symbol, FILE *out);

/*---------------------------------------------------------------------------*/

/**
 * @brief Create a full_symbol set, that will be used to do gaussian elimination
 */
swif_full_symbol_set_t *full_symbol_set_alloc(void);

/**
 * @brief Free a full_symbol set
 */
void full_symbol_set_free(swif_full_symbol_set_t *set);


/**
 * @brief Add a full_symbol to a packet set.
 * 
 * Gaussian elimination can occur.
 * Teturn the pivot associated to the new full_symbol 
 * or SYMBOL_ID_NONE if dependent (e.g. redundant) packet
 */
uint32_t full_symbol_set_add
(swif_full_symbol_set_t *set, swif_full_symbol_t *full_symbol);

static inline bool full_symbol_includes_id(swif_full_symbol_t* symbol,
                                           symbol_id_t id);

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __SWIF_FULL_SYMBOL_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
