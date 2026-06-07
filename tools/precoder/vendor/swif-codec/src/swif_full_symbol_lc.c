
/**
 * @brief       For now this is an hackish adaptation on top of liblc,
 *              will be replaced by a proper implementation later.
 */


#include "coded-packet.h"
#include "packet-set.h"

#define LIBLC_GF256 3
#define COEF_PER_HEADER ((COEF_HEADER_SIZE*BITS_PER_BYTE)>>LIBLC_GF256)

#include "swif_full_symbol.h"

/*---------------------------------------------------------------------------*/

struct s_swif_full_symbol_t {
    coded_packet_t* coded_packet;
    uint32_t symbol_size;
};

/*---------------------------------------------------------------------------*/

struct s_swif_full_symbol_set_t {
    packet_set_t     packet_set;
    reduction_stat_t reduction_stat;
};

/*---------------------------------------------------------------------------*/


/**
 * @brief Create a full_symbol set, that will be used to do gaussian elimination
 */
swif_full_symbol_set_t *full_symbol_set_alloc()
{
    swif_full_symbol_set_t *result
        = (swif_full_symbol_set_t *)calloc(1, sizeof(swif_full_symbol_set_t));
    if (result == NULL) {
        return NULL;
    }

    packet_set_init(&result->packet_set, LIBLC_GF256, NULL, NULL, NULL, NULL);
    return result;
}

/**
 * @brief Free a full_symbol set
 */
void full_symbol_set_free(swif_full_symbol_set_t *set)
{
    memset(&set->packet_set, 0, sizeof(packet_set_t));
    free(set);
}


/**
 * @brief Add a full_symbol to a packet set.
 * 
 * Gaussian elimination can occur.
 * Return the pivot associated to the new full_symbol 
 * or SYMBOL_ID_NONE if dependent (e.g. redundant) packet
 * 
 * The full_symbol is not freed and also reference is not captured.
 */
uint32_t swif_full_symbol_set_add
(swif_full_symbol_set_t* set, swif_full_symbol_t* full_symbol)
{
    uint32_t packet_id = packet_set_add(&set->packet_set,
                                        full_symbol->coded_packet,
                                        &set->reduction_stat, false);
    if (packet_id != PACKET_ID_NONE) {
        return packet_id;
    } else {
        return SYMBOL_ID_NONE;
    }
}

/*---------------------------------------------------------------------------*/

/**
 * @brief Create a full_symbol from a raw packet (a set of bytes)
 *        and initialize it with content '0'
 */
swif_full_symbol_t *full_symbol_alloc
(uint32_t symbol_id_size, uint32_t symbol_size)
{
    assert(symbol_id_size <= COEF_HEADER_SIZE);
    assert(symbol_size <= CODED_PACKET_SIZE);
    swif_full_symbol_t *result
        = (swif_full_symbol_t *)calloc(1, sizeof(swif_full_symbol_t));
    if (result == NULL) {
        return NULL;
    }
  
    coded_packet_t *coded_packet
        = (coded_packet_t *)calloc(1, sizeof(coded_packet_t));
    if (coded_packet == NULL) {
        free(result);
        return NULL;
    }

    result->coded_packet = coded_packet;
    result->symbol_size = symbol_size;
    coded_packet_init(result->coded_packet, LIBLC_GF256);
    return result;
}


/**
 * @brief Create a full_symbol from a raw packet (a set of bytes)
 *        and initialize it with content '0'
 */
swif_full_symbol_t *full_symbol_create_from_source
(uint32_t symbol_id, uint8_t *symbol_data, uint32_t symbol_size)
{
    swif_full_symbol_t *result = full_symbol_alloc(1, symbol_size);
    if (result == NULL) {
        return NULL;
    }
    coded_packet_set_coef(result->coded_packet, symbol_id, 1);
    memcpy(coded_packet_data(result->coded_packet), symbol_data, symbol_size);
    result->symbol_size = symbol_size;
    coded_packet_adjust_min_max_coef(result->coded_packet);
    return result;
}

swif_full_symbol_t *full_symbol_create
(uint8_t* symbol_coef_table, uint32_t min_symbol_id, uint32_t nb_symbol_id,
 uint8_t* symbol_data, uint32_t symbol_size)
{
    swif_full_symbol_t *result = full_symbol_alloc(nb_symbol_id, symbol_size);
    if (result == NULL) {
        return NULL;
    }
    for (uint32_t i=0; i<nb_symbol_id; i++) {
        /* Note: assumption of GF256 in next line -> 1 coef 1 byte: */ 
        uint8_t coef = symbol_coef_table[i]; 
        coded_packet_set_coef(result->coded_packet, i+min_symbol_id, coef);
    }
    memcpy(coded_packet_data(result->coded_packet), symbol_data, symbol_size);
    coded_packet_adjust_min_max_coef(result->coded_packet);
    result->symbol_size = symbol_size;
    return result;
}

/**
 * @brief Release a full_symbol
 */
void full_symbol_free(swif_full_symbol_t* full_symbol)
{
    assert(full_symbol != NULL);
    assert(full_symbol->coded_packet != NULL);
    free(full_symbol->coded_packet);
    full_symbol->coded_packet = NULL;
    free(full_symbol);
}

/**
 * @brief Create a new (unlinked) copy of a full_symbol
 */
swif_full_symbol_t *full_symbol_clone(swif_full_symbol_t* full_symbol)
{
    swif_full_symbol_t *result = full_symbol_alloc(
        COEF_HEADER_SIZE, full_symbol->symbol_size);
    memcpy(result->coded_packet, full_symbol->coded_packet,
           sizeof(coded_packet_t));
    return result;
}

/**
 * @brief get the size of the data
 */
uint32_t full_symbol_get_size(swif_full_symbol_t *full_symbol)
{ return full_symbol->symbol_size; }

/**
 * @brief get the minimum source index that appears in the symbol
 *        returns SYMBOL_ID_NONE if there is none (e.g. symbol is 0)
 */
uint32_t full_symbol_get_min_symbol_id(swif_full_symbol_t *full_symbol)
{
    coded_packet_adjust_min_max_coef(full_symbol->coded_packet);
    if (full_symbol->coded_packet->coef_pos_min != COEF_POS_NONE) {
        return full_symbol->coded_packet->coef_pos_min;
    } else {
        return SYMBOL_ID_NONE;        
    }
}

/**
 * @brief get the maximum source index that appears in the symbol
 *        returns SYMBOL_ID_NONE if there is none (e.g. symbol is 0)
 */
uint32_t full_symbol_get_max_symbol_id(swif_full_symbol_t *full_symbol)
{
    coded_packet_adjust_min_max_coef(full_symbol->coded_packet);
    if (full_symbol->coded_packet->coef_pos_max != COEF_POS_NONE) {
        return full_symbol->coded_packet->coef_pos_max;
    } else {
        return SYMBOL_ID_NONE;        
    }
}

/**
 * @brief get the coefficient corresponding to the specified symbol identifier
 */
uint8_t full_symbol_get_coef
(swif_full_symbol_t *full_symbol, uint32_t symbol_id)
{
    return coded_packet_get_coef(full_symbol->coded_packet, symbol_id);
}

/**
 * @brief get the symbol 'data'. result_data should be a pointer to a
 *        a block of memory of full_symbol_get_size(full_symbol)
 */
void full_symbol_get_data
(swif_full_symbol_t *full_symbol, uint8_t *result_data)
{
    memcpy(result_data, coded_packet_data(full_symbol->coded_packet),
           full_symbol->symbol_size);
}

void full_symbol_dump(swif_full_symbol_t *full_symbol, FILE *out)
{
    fprintf(out, "{'size':%u, 'internal':", full_symbol->symbol_size);
    coded_packet_pywrite(out, full_symbol->coded_packet);
    fprintf(out, "}");    
}

/*---------------------------------------------------------------------------*/
