#ifndef __SWIF_FULL_SYMBOL_IMPL_H__
#define __SWIF_FULL_SYMBOL_IMPL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* XXX: move in more general header file */
#define WARNING_PRINT(fmt, args...) fprintf(stderr, "WARNING: %s: %d: %s(): " fmt, \
    __FILE__, __LINE__, __func__, ##args)
//#define WITH_DEBUG
#ifdef WITH_DEBUG
#define IF_DEBUG(...) do { __VA_ARGS__; } while(0)
#else
#define IF_DEBUG(...) do { } while(0)
#endif

/*---------------------------------------------------------------------------*/
/**
 * @brief       Implementation of full symbol operations.
 */


/*---------------------------------------------------------------------------*/

/* The implementation of full_symbol_t, a linear combination of source packets.
 * - `coef' represents the encoding header (coefficients) of the combination
 * - `coef[0]' corresponds to the source symbol of index `first_id'
 * - `last_id' (included) is the last possible source symbol included in the
 * array `coef', and thus defines the array size
 * - `first_nonzero_id' and `last_nonzero_id' correspond the first and last
 * source symbol indices that have non zero coefficients. SYMBOL_ID_NONE is
 * used when the full_symbol_t corresponds to zero, and these values are 
 * maintained after each operation on a packet.
 * - `data' is the coded symbol itself
 * - `data_size' is the size of the coded symbol.
 */
struct s_swif_full_symbol_t {
    /* first_id, last_id  relate to the memory allocation
       can be SYMBOL_ID_NONE (when no coefficient available) */
    uint8_t *coef; /* never NULL */
    symbol_id_t first_id; /* coef[0] is the coefficient of first_id */
    symbol_id_t last_id;  /* (last included) */

    /* first_nonzero_id, last_nonzero_id relate to the coef array
       they can be SYMBOL_ID_NONE if no coef is different of zero
       otherwise the coefficients at these, are always non-zero
       note: full_symbol_get_coef(i) is 0 outside of this range */
    symbol_id_t first_nonzero_id;
    symbol_id_t last_nonzero_id;    
    
    uint8_t *data; /* the coded symbol itself (the data) - never NULL */
    uint32_t data_size; /* size of the coded symbol */
};

/*---------------------------------------------------------------------------*/

typedef void (*notify_decoded_func_t)(swif_full_symbol_set_t *set, symbol_id_t symbol_id, void *context);
  
struct s_swif_full_symbol_set_t {
    uint32_t size; /* size of the table containing pointers to full_symbol */
    uint32_t first_symbol_id; /* the index of full_symbol_tab */
    uint32_t nmbr_packets;
    swif_full_symbol_t **full_symbol_tab;

    notify_decoded_func_t notify_decoded_func; /* XXX: manually set */
    void                 *notify_context;
};

/* The following constant is used to declare that an entry is not used */
#define ENTRY_INDEX_NONE 0xfffffffful 
    
/*---------------------------------------------------------------------------*/

/**
 * @brief Add a full_symbol to a packet set.
 * 
 * Online Gaussian elimination is performed.
 * Return the pivot associated to the new full_symbol (as source symbol index)
 * or ENTRY_INDEX_NONE if dependent (e.g. redundant) packet
 * 
 * The full_symbol is not freed and also reference is not captured.
 */
symbol_id_t swif_full_symbol_set_add
(swif_full_symbol_set_t* set, swif_full_symbol_t* full_symbol);

/**
 * @brief Create a full_symbol from a raw packet (a set of bytes)
 *        and initialize it with content '0'
 */
swif_full_symbol_t *full_symbol_alloc
(symbol_id_t first_symbol_id, symbol_id_t last_symbol_id, uint32_t symbol_size);

/**
 * @brief Dump the content of a full_symbol set for debugging purposes
 *        
 */
void full_symbol_set_dump(swif_full_symbol_set_t *full_symbol_set, FILE *out);

/**
 * @brief get the size of the data
 */



static inline bool full_symbol_has_sufficient_size(swif_full_symbol_t* symbol,
                                           symbol_id_t id1, symbol_id_t id2);

static inline bool full_symbol_includes_id(swif_full_symbol_t* symbol,
                                           symbol_id_t symbol_id);
// adjust full symbol min coef


static bool full_symbol_adjust_min_coef(swif_full_symbol_t* symbol);




// adjust full symbol max coef

static bool full_symbol_adjust_max_coef(swif_full_symbol_t* symbol);


// adjust full symbol min and max coef
// Recompute the first_nonzero_id and last_nonzero_id 
bool full_symbol_adjust_min_max_coef(swif_full_symbol_t* symbol);


/*---------------------------------------------------------------------------*/

/**
 * @brief Take a symbol and add another symbol multiplied by a 
 *        coefficient, e.g. performs the equivalent of: p1 += coef * p1
 * @param[in,out] p1     First symbol (to which coef*p1 will be added)
 * @param[in]     coef2  Coefficient by which the second packet is multiplied
 */
void full_symbol_scale
( swif_full_symbol_t *symbol1, uint8_t coereef);


/**
 * @brief Take a symbol and add another symbol to it, e.g. performs the equivalent of: p3 = p1 + p2
 * @param[in] p1     First symbol (to which p2 will be added)
 * @param[in] p2     Second symbol
 * @param[in] p3     result XXX
 */
void full_symbol_add_base(swif_full_symbol_t *symbol1, swif_full_symbol_t *symbol2, swif_full_symbol_t *symbol_result);



swif_full_symbol_t* full_symbol_add
(swif_full_symbol_t *symbol1, swif_full_symbol_t *symbol2);


/*---------------------------------------------------------------------------*/

void full_symbol_dump(swif_full_symbol_t *full_symbol, FILE *out);

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CODED_PACKET_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
