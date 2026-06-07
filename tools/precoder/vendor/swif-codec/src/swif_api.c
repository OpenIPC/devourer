/**
 * SWiF Codec: an open-source sliding window FEC codec in C
 * https://github.com/irtf-nwcrg/swif-codec
 */

/* PATCHED: removed `#define DEBUG 1` to silence verbose codec output. See PATCHES.md. */
#include "swif_includes.h"
#include "swif_rlc_api.c"


/*******************************************************************************
 * Encoder functions
 */


/**
 * Create and initialize an encoder, providing only key parameters.
 **/
swif_encoder_t* swif_encoder_create (
                                swif_codepoint_t codepoint,
                                uint32_t        verbosity,
                                uint32_t        symbol_size,
                                uint32_t        max_coding_window_size)
{
    /* initialize the encoder */
    switch (codepoint) {
        case SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC:
            return (swif_rlc_encoder_create(codepoint, verbosity, symbol_size, max_coding_window_size));
        default:
            fprintf(stderr, "Error, swif_encoder_create: codepoint not recognized\n");
            return NULL;
    }
}


/**
 * Release an encoder and its associated ressources.
 **/
swif_status_t   swif_encoder_release (swif_encoder_t*        enc)
{
    /* initialize the encoder */
    switch (enc->codepoint) {
        case SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC:
            return (swif_rlc_encoder_release(enc));
        default:
            fprintf(stderr, "Error, swif_encoder_release: codepoint not recognized\n");
            return SWIF_STATUS_ERROR;
    }
}


/**
 * Set the various callback functions for this encoder.
 * All the callback functions require an opaque context parameter, that must be
 * initialized accordingly by the application, since it is application specific.
 */
swif_status_t   swif_encoder_set_callback_functions (
                swif_encoder_t*        enc,
                void (*source_symbol_removed_from_coding_window_callback) (
                                        void*   context,
                                        esi_t   old_symbol_esi),
                void* context_4_callback)
{
    return (*enc->set_callback_functions)(enc, source_symbol_removed_from_coding_window_callback, context_4_callback);
}

/**
 * This function sets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_encoder_set_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value)
{
    return (*enc->set_parameters)(enc, type, length, value);
}

/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_encoder_get_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value)
{
    return (*enc->get_parameters)(enc, type, length, value);
}


/**
 * Create a single repair symbol (i.e. perform an encoding).
 */
swif_status_t   swif_build_repair_symbol (
                                swif_encoder_t* enc,
                                void**          new_buf)
{
    return (*enc->build_repair_symbol)(enc, new_buf);
}


/*******************************************************************************
 * Decoder functions
 */


/**
 * Create and initialize a decoder, providing only key parameters.
 */
swif_decoder_t* swif_decoder_create (
                                swif_codepoint_t codepoint,
                                uint32_t        verbosity,
                                uint32_t        symbol_size,
                                uint32_t        max_coding_window_size,
                                uint32_t        max_linear_system_size)
{
    /* initialize the decoder */
    switch (codepoint) {
        case SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC:
            return (swif_rlc_decoder_create(codepoint, verbosity, symbol_size, max_coding_window_size, max_linear_system_size));
        default:
            fprintf(stderr, "Error, swif_decoder_create: codepoint not recognized\n");
            return NULL;
    }
}


/**
 * Release a decoder and its associated ressources.
 **/
swif_status_t   swif_decoder_release (swif_decoder_t*        dec)
{
    switch (dec->codepoint) {
        case SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC:
            return (swif_rlc_decoder_release(dec));
        default:
            fprintf(stderr, "Error, swif_decoder_release: codepoint not recognized\n");
            return SWIF_STATUS_ERROR;
    }
}


/**
 * Set the various callback functions for this decoder.
 * All the callback functions require an opaque context parameter, that
 * must be initialized accordingly by the application, since it is
 * application specific.
 */
swif_status_t   swif_decoder_set_callback_functions (
                swif_decoder_t*  dec,
                void (*source_symbol_removed_from_linear_system_callback) (
                                        void*   context,
                                        esi_t   old_symbol_esi),
                void* (*decodable_source_symbol_callback) (
                                        void    *context,
                                        esi_t   esi),
                void* (*decoded_source_symbol_callback) (
                                        void    *context,
                                        void    *new_symbol_buf,
                                        esi_t   esi),
                void*        context_4_callback)
{
    return (*dec->set_callback_functions)(dec, source_symbol_removed_from_linear_system_callback,
            decodable_source_symbol_callback, decoded_source_symbol_callback,
            context_4_callback);
}


/**
 * This function sets one or more FEC codec specific parameters,
 *        using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_decoder_set_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value)
{
    return (*dec->set_parameters)(dec, type, length, value);
}


/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_decoder_get_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value)
{
    return (*dec->get_parameters)(dec, type, length, value);
}


/**
 * Submit a received source symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 */
swif_status_t   swif_decoder_decode_with_new_source_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_symbol_esi)
{
    return (*dec->decode_with_new_source_symbol)(dec, new_symbol_buf, new_symbol_esi);
}


/**
 * Submit a received repair symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 */
swif_status_t   swif_decoder_decode_with_new_repair_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_src_symbol_esi)
{
    return (*dec->decode_with_new_repair_symbol)(dec, new_symbol_buf, new_src_symbol_esi);
}


/*******************************************************************************
 * Coding Window Functions at an Encoder and Decoder
 */


/**
 * This function resets the current coding window. We assume here that
 * this window is maintained by the FEC codec instance.
 * Encoder:     reset the encoding window for the encoding of future
 *              repair symbols.
 * Decoder:     reset the coding window under preparation associated to
 *              a repair symbol just received.
 */
swif_status_t   swif_encoder_reset_coding_window (swif_encoder_t*  enc)
{
    return (*enc->reset_coding_window)(enc);
}

swif_status_t   swif_decoder_reset_coding_window (swif_decoder_t*  dec)
{
    return (*dec->reset_coding_window)(dec);
}

/**
 * Add this source symbol to the coding window.
 * Encoder:     add a source symbol to the coding window.
 * Decoder:     add a source symbol to the coding window under preparation.
 */
swif_status_t   swif_encoder_add_source_symbol_to_coding_window (
                                swif_encoder_t* enc,
                                void*           new_src_symbol_buf,
                                esi_t           new_src_symbol_esi)
{
/* ajouter un élément à la liste chainée des symboles sources
retirer le symbole source le plus ancien si l'EW déborde. */
    return (*enc->add_source_symbol_to_coding_window)(enc, new_src_symbol_buf, new_src_symbol_esi);
}

swif_status_t   swif_decoder_add_source_symbol_to_coding_window (
                                swif_decoder_t* dec,
                                esi_t           new_src_symbol_esi)
{
    return (*dec->add_source_symbol_to_coding_window)(dec, new_src_symbol_esi);
}


/**
 * Remove this source symbol from the coding window.
 */
swif_status_t   swif_encoder_remove_source_symbol_from_coding_window (
                                swif_encoder_t* enc,
                                esi_t           old_src_symbol_esi)
{
    return (*enc->remove_source_symbol_from_coding_window)(enc, old_src_symbol_esi);
}

swif_status_t   swif_decoder_remove_source_symbol_from_coding_window (
                                swif_decoder_t* dec,
                                esi_t           old_src_symbol_esi)
{
    return (*dec->remove_source_symbol_from_coding_window)(dec, old_src_symbol_esi);
}


/**
 * Get information on the current coding window at the encoder.
 */
swif_status_t   swif_encoder_get_coding_window_information (
                                swif_encoder_t* enc,
                                esi_t*          first,
                                esi_t*          last,
                                uint32_t*       nss)
{
    return (*enc->get_coding_window_information)(enc, first, last, nss);
}


/*******************************************************************************
 * Coding Coefficients Functions at an Encoder and Decoder
 */


/**
 * Encoder: this function specifies the coding coefficients chosen by
 *          the application if this is the way the codec works.
 * Decoder: communicate with this function the coding coefficients
 *          associated to a repair symbol and carried in the packet
 *          header.
 */
swif_status_t   swif_encoder_set_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void*           coding_coefs_tab,
                                uint32_t        nb_coefs_in_tab)
{
    return (*enc->set_coding_coefs_tab)(enc, coding_coefs_tab, nb_coefs_in_tab);
}


swif_status_t   swif_decoder_set_coding_coefs_tab (
                                swif_decoder_t* dec,
                                void*           coding_coefs_tab,
                                uint32_t        nb_coefs_in_tab)
{
    return (*dec->set_coding_coefs_tab)(dec, coding_coefs_tab, nb_coefs_in_tab);
}


/**
 * The coding coefficients may be generated in a deterministic manner,
 * for instance by a PRNG known by the codec and a seed (perhaps with
 * other parameters) provided by the application.
 * The codec may also choose in an autonomous manner these coefficients.
 * This function is used to trigger this process.
 * When the choice is made in an autonomous manner, the actual coding
 * coefficient or key used by the codec can be retrieved with
 * swif_encoder_get_coding_coefs_tab().
 */
swif_status_t   swif_encoder_generate_coding_coefs (
                                swif_encoder_t* enc,
                                uint32_t        key,
                                uint32_t        add_param)
{
    return (*enc->generate_coding_coefs)(enc, key, add_param);
}

swif_status_t   swif_decoder_generate_coding_coefs (
                                swif_decoder_t* dec,
                                uint32_t        key,
                                uint32_t        add_param)
{
    return (*dec->generate_coding_coefs)(dec, key, add_param);
}


/**
 * This function enables the application to retrieve the set of coding
 * coefficients generated and used by build_repair_symbol(). This is
 * useful when the choice of coefficients is performed by the codec in
 * an autonomous manner but needs to be sent in the repair packet header.
 * This function is only used by an encoder.
 */
swif_status_t   swif_encoder_get_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void**          coding_coefs_tab,
                                uint32_t*       nb_coefs_in_tab)
{
    return (*enc->get_coding_coefs_tab)(enc, coding_coefs_tab, nb_coefs_in_tab);
}


