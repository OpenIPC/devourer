#ifndef SWIF_RLC_API_H
#define SWIF_RLC_API_H
#include "swif_includes.h"


/*******************************************************************************
 * Encoder functions
 */


/**
 * Release an encoder and its associated ressources.
 **/
swif_status_t   swif_rlc_encoder_release (swif_encoder_t*        enc);


/**
 * Set the various callback functions for this encoder.
 * All the callback functions require an opaque context parameter, that must be
 * initialized accordingly by the application, since it is application specific.
 */
swif_status_t   swif_rlc_encoder_set_callback_functions (
                swif_encoder_t*        enc,
                void (*source_symbol_removed_from_coding_window_callback) (
                                        void*   context,
                                        esi_t   old_symbol_esi),
                void* context_4_callback);


/**
 * This function sets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_rlc_encoder_set_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);

/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_rlc_encoder_get_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);


/**
 * Create a single repair symbol (i.e. perform an encoding).
 */
swif_status_t   swif_rlc_build_repair_symbol (
                                swif_encoder_t* generic_encoder,
                                void**           new_buf);


/*******************************************************************************
 * Decoder functions
 */


/**
 * Release a decoder and its associated ressources.
 **/
swif_status_t   swif_rlc_decoder_release (swif_decoder_t*        dec);


/**
 * Set the various callback functions for this decoder.
 * All the callback functions require an opaque context parameter, that
 * must be initialized accordingly by the application, since it is
 * application specific.
 */
swif_status_t   swif_rlc_decoder_set_callback_functions (
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
                void*        context_4_callback);


/**
 * This function sets one or more FEC codec specific parameters,
 *        using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_rlc_decoder_set_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);


/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 */
swif_status_t   swif_rlc_decoder_get_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);


/**
 * Submit a received source symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 */
swif_status_t   swif_rlc_decoder_decode_with_new_source_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_symbol_esi);


/**
 * Submit a received repair symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 */
swif_status_t   swif_rlc_decoder_decode_with_new_repair_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_symbol_esi);

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
swif_status_t   swif_rlc_encoder_reset_coding_window (swif_encoder_t*  enc);

swif_status_t   swif_rlc_decoder_reset_coding_window (swif_decoder_t*  dec);

/**
 * Add this source symbol to the coding window.
 * Encoder:     add a source symbol to the coding window.
 * Decoder:     add a source symbol to the coding window under preparation.
 */
swif_status_t   swif_rlc_encoder_add_source_symbol_to_coding_window (
                                swif_encoder_t* generic_enc,
                                void*           new_src_symbol_buf,
                                esi_t           new_src_symbol_esi);

swif_status_t   swif_rlc_decoder_add_source_symbol_to_coding_window (
                                swif_decoder_t* dec,
                                esi_t           new_src_symbol_esi);


/**
 * Remove this source symbol from the coding window.
 */
swif_status_t   swif_rlc_encoder_remove_source_symbol_from_coding_window (
                                swif_encoder_t* enc,
                                esi_t           old_src_symbol_esi);

swif_status_t   swif_rlc_decoder_remove_source_symbol_from_coding_window (
                                swif_decoder_t* dec,
                                esi_t           old_src_symbol_esi);


/**
 * Get information on the current coding window at the encoder.
 */
swif_status_t   swif_rlc_encoder_get_coding_window_information (
                                swif_encoder_t* generic_encoder,
                                esi_t*          first,
                                esi_t*          last,
                                uint32_t*       nss);


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
swif_status_t   swif_rlc_encoder_set_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void*           coding_coefs_tab,
                                uint32_t        nb_coefs_in_tab);


swif_status_t   swif_rlc_decoder_set_coding_coefs_tab (
                                swif_decoder_t* dec,
                                void*           coding_coefs_tab,
                                uint32_t        nb_coefs_in_tab);

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
swif_status_t   swif_rlc_encoder_generate_coding_coefs (
                                swif_encoder_t* enc,
                                uint32_t        key,
                                uint32_t        add_param);


swif_status_t   swif_rlc_decoder_generate_coding_coefs (
                                swif_decoder_t* dec,
                                uint32_t        key,
                                uint32_t        add_param);


/**
 * This function enables the application to retrieve the set of coding
 * coefficients generated and used by build_repair_symbol(). This is
 * useful when the choice of coefficients is performed by the codec in
 * an autonomous manner but needs to be sent in the repair packet header.
 * This function is only used by an encoder.
 */
swif_status_t   swif_rlc_encoder_get_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void**          coding_coefs_tab,
                                uint32_t*       nb_coefs_in_tab);


/*******************************************************************************
 * Session creation functions (last position to avoid compilation errors for
 * codec specific functions and avoid the need to add prototypes in header).
 */

/**
 * Create and initialize an encoder, providing only key parameters.
 **/
swif_encoder_t* swif_rlc_encoder_create (swif_codepoint_t codepoint,
                                         uint32_t        verbosity,
                                         uint32_t        symbol_size,
                                         uint32_t        max_coding_window_size);


/**
 * Create and initialize a decoder, providing only key parameters.
 */
swif_decoder_t* swif_rlc_decoder_create (
                                swif_codepoint_t codepoint,
                                uint32_t        verbosity,
                                uint32_t        symbol_size,
                                uint32_t        max_coding_window_size,
                                uint32_t        max_linear_system_size);
#endif
