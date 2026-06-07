/**
 * @defgroup    SWiF-Codec-API
 * @ingroup     swif-codec-api
 * @brief       an open-source sliding window FEC codec in C (API)
 * @{
 *
 * @file        
 * @brief       The Generic FEC API
 *
 */

/**
 * SWiF Codec: an open-source sliding window FEC codec in C
 * https://github.com/irtf-nwcrg/swif-codec
 */
#include <stdint.h>
#ifndef __SWIF_API_H__
#define __SWIF_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*/


/**
 * Return value of any function.
 *
 * SWIF_STATUS_OK = 0   Success
 * SWIF_STATUS_FAILURE  Failure. The function called did not succeed to
 *                      perform its task, however this is not an error
 *                      (e.g., it happens when decoding fails).
 * SWIF_STATUS_ERROR    Generic error type. The detailed error type is
 *                      stored in the errno variable of swif_encoder_t and
 *                      swif_decoder_t structures.
 */
typedef enum {
        SWIF_STATUS_OK = 0,
        SWIF_STATUS_FAILURE,
        SWIF_STATUS_ERROR
} swif_status_t;


/**
 * Potential errors.
 */
typedef enum {
        SWIF_ERRNO_NULL = 0,            /* everything is fine */
        SWIF_ERRNO_UNSUPPORTED_CODEPOINT,
        /* and many more... */
} swif_errno_t;


/**
 * FEC Codepoints.
 * These identifiers are opaque identifiers that fully identify an FEC
 * code locally, including certain parameters like its Galois Field, or
 * the coding coefficient generator (if several exist).
 * These codepoints are codec specific and only have a local meaning.
 * They should not be transmitted as different implementations may use
 * them inconsistently.
 * Note that the same FEC code may be used by several FEC Encoding IDs
 * and therefore share the same codepoint. On the opposite multiple
 * implementations of a given FEC code may exist locally, for instance
 * with different optimizations, and then several codepoints, one per
 * codec, will exist for the same FEC code. The following names are
 * therefore only provided as examples.
 */
typedef enum {
        SWIF_CODEPOINT_NULL = 0,        /* codepoint 0 is reserved */

        /* codepoint for RLC sliding window code, GF(2^8) and full
         * density (simplified version of FECFRAME FEC Enc. ID XXX). */
        SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC

        /* list here other identifiers for any codec of interest... */
} swif_codepoint_t;


/**
 * Encoding Symbol Identifier (ESI) generic type.
 * With Sliding Window FEC codes, an ESI is in fact a source symbol
 * identifier, unlike block FEC codes.
 */
typedef uint32_t    esi_t;

/* Unused/invalid esi value. Must be coherent with esi_t definition */
#define INVALID_ESI	0xFFFFFFFF


/*******************************************************************************
 * Encoder functions
 */

/*
 * TODO: it would make sense to hide the functions (and even
 * other attributes ?) to the above application so as the
 * application can only interact through the defined API.
 */

/**
 * Throughout the API, a pointer to this structure is used as an
 * identifier of the encoder instance (or "enc").
 *
 * This generic structure is meant to be extended by each codec with
 * new pieces of information that are specific to each codec.
 */
typedef struct swif_encoder {
	swif_codepoint_t	codepoint;

	/* when a function returns with SWIF_STATUS_ERROR, the errno
	 * variable contains a more detailed error type. This variable
	 * is set by the codec and accessible to the application in
	 * READ ONLY mode. Otherwise its value is undefined. */
	swif_errno_t		swif_errno;

	/* pointers to codec specific versions of API functions. */
	swif_status_t   (*set_callback_functions) (
		struct swif_encoder*, void (*) (void*, esi_t), void*);
	swif_status_t   (*set_parameters)  (
		struct swif_encoder*, uint32_t, uint32_t, void*);
	swif_status_t   (*get_parameters)  (
		struct swif_encoder*, uint32_t, uint32_t, void*);
	swif_status_t   (*build_repair_symbol) (
		struct swif_encoder*, void**);
	swif_status_t   (*reset_coding_window) (struct swif_encoder*);
	swif_status_t   (*add_source_symbol_to_coding_window) (
		struct swif_encoder*, void*, esi_t);
	swif_status_t   (*remove_source_symbol_from_coding_window) (
		struct swif_encoder*, esi_t);
	swif_status_t	(*get_coding_window_information) (
		struct swif_encoder*, esi_t*, esi_t*, uint32_t*);
	swif_status_t   (*set_coding_coefs_tab) (
		struct swif_encoder*, void*, uint32_t);
	swif_status_t   (*generate_coding_coefs) (
		struct swif_encoder*, uint32_t, uint32_t);
	swif_status_t   (*get_coding_coefs_tab) (
		struct swif_encoder*, void**, uint32_t*);
} swif_encoder_t;


/**
 * Create and initialize an encoder, providing only key parameters.
 *
 * @param codepoint     opaque identifier that fully identifies the FEC
 *                      code to use.
 * @param verbosity     print information on the codec processing.
 *                      0 is the minimum verbosity, the maximum verbosity
 *                      level being implementation specific.
 * @param symbol_size   source and repair symbol size in bytes. Cannot
 *                      change during the codec instance lifetime.
 * @param max_encoding_window_size
 * @return              pointer to a swif_encoder_t structure if okay, or
 *                      NULL in case of error.
 **/
swif_encoder_t* swif_encoder_create (
                                swif_codepoint_t codepoint,
                                uint32_t        verbosity,
                                uint32_t        symbol_size,
                                uint32_t        max_coding_window_size);


/**
 * Release an encoder and its associated ressources.
 **/
swif_status_t   swif_encoder_release (swif_encoder_t*        enc);


/**
 * Set the various callback functions for this encoder.
 * All the callback functions require an opaque context parameter, that
 * must be initialized accordingly by the application, since it is
 * application specific.
 *
 * @param enc
 * @param source_symbol_removed_from_coding_window_callback
 *              (IN) Pointer to the function, within the application,
 *              that needs to be called each time a source symbol is
 *              removed from the left side of the coding window.
 *              This callback is called each time the encoding window
 *              slides to the right and an old source symbol needs to
 *              be removed on the left. The application therefore knows
 *              this source symbol will no longer be used by the codec
 *              and can free the associated buffer if need be. This
 *              function does not return anything.
 * @param context_4_callback
 *              (IN) Pointer to the application-specific context that
 *              will be passed to the callback function (if any). This
 *              context is not interpreted by this function.
 * @return
 */
swif_status_t   swif_encoder_set_callback_functions (
                swif_encoder_t*         enc,
                void (*source_symbol_removed_from_coding_window_callback) (
                                        void*   context,
                                        esi_t   old_symbol_esi),
                void* context_4_callback);

/**
 * This function sets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 *
 * @param enc
 * @param type          (IN) Type of parameter.
 * @param length        (IN) length of the pointed value.
 * @param value         (IN) Pointer to the value. The exact type of
 *                      the object pointed is FEC codec specific.
 * @return
 */
swif_status_t   swif_encoder_set_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);

/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 *
 * @param enc
 * @param type          (IN) Type of parameter.
 * @param length        (IN) length of the pointed value.
 * @param value         (IN/OUT) Pointer to the value. The exact type of
 *                      the object pointed is FEC codec specific.
 *                      This function updates the value object
 *                      accordingly. The caller, who knows the FEC codec,
 *                      is responsible to allocate the appropriate
 *                      object buffer.
 * @return
 */
swif_status_t   swif_encoder_get_parameters  (
                                swif_encoder_t* enc,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);

/**
 * List here the FEC codec specific control parameters.
 */
enum {
        swif_ENCODER_GET_PARAM_ENCODER_STATISTICS = 1,
        swif_ENCODER_SET_PARAM_RLC_DENSITY_THRESHOLD
};


/**
 * Create a single repair symbol (i.e. perform an encoding).
 *
 * @param new_buf       (IN/OUT) Address of a pointer to the buffer for
 *			the repair symbol to build. The pointer can
 *			either point to a buffer allocated by the
 *			application, or be set to NULL meaning that
 *			this function will allocate memory and initialize
 *			this pointer accordingly.
 * @return
 */
swif_status_t   swif_build_repair_symbol (
                                swif_encoder_t* enc,
                                void**          new_buf);


/*******************************************************************************
 * Decoder functions
 */


/**
 * Throughout the API, a pointer to this structure is used as an
 * identifier of the decoder instance (or "dec").
 *
 * This generic structure is meant to be extended by each codec with
 * new pieces of information that are specific to each codec.
 */
typedef struct swif_decoder {
	swif_codepoint_t	codepoint;

	/* when a function returns with SWIF_STATUS_ERROR, the errno
	 * variable contains a more detailed error type. This variable
	 * is set by the codec and accessible to the application in
	 * READ ONLY mode. Otherwise its value is undefined. */
	swif_errno_t		swif_errno;

	/* pointers to codec specific versions of API functions. */
	swif_status_t   (*set_callback_functions) (
		struct swif_decoder*, void (*) (void*, esi_t),
		void* (*) (void*, esi_t),
		void* (*) (void*, void*, esi_t), void*);
	swif_status_t   (*set_parameters)  (
		struct swif_decoder*, uint32_t, uint32_t, void*);
	swif_status_t   (*get_parameters)  (
		struct swif_decoder*, uint32_t, uint32_t, void*);
	swif_status_t   (*decode_with_new_source_symbol) (
		struct swif_decoder*, void* const, esi_t);
	swif_status_t   (*decode_with_new_repair_symbol) (
		struct swif_decoder*, void* const, esi_t);
	swif_status_t   (*reset_coding_window) (struct swif_decoder*);
	swif_status_t   (*add_source_symbol_to_coding_window) (
		struct swif_decoder*, esi_t);
	swif_status_t   (*remove_source_symbol_from_coding_window) (
		struct swif_decoder*, esi_t);
	swif_status_t   (*set_coding_coefs_tab) (
		struct swif_decoder*, void*, uint32_t);
	swif_status_t   (*generate_coding_coefs) (
		struct swif_decoder*, uint32_t, uint32_t);
} swif_decoder_t;


/**
 * Create and initialize a decoder, providing only key parameters.
 *
 * @param codepoint     opaque identifier that fully identifies the FEC
 *                      code to use.
 * @param verbosity     print information on the codec processing.
 *                      0 is the minimum verbosity, the maximum verbosity
 *                      level being implementation specific.
 * @param symbol_size   source and repair symbol size in bytes. Cannot
 *                      change during the codec instance lifetime.
 * @param max_coding_window_size
 * @param max_linear_system_size
 * @return              pointer to a swif_decoder_t structure if okay, or
 *                      NULL in case of error.
 **/
swif_decoder_t* swif_decoder_create (
                                swif_codepoint_t codepoint,
                                uint32_t        verbosity,
                                uint32_t        symbol_size,
                                uint32_t        max_coding_window_size,
                                uint32_t        max_linear_system_size);


/**
 * Release a decoder and its associated ressources.
 *
 * @param dec        context (i.e., pointer to decoder structure).
 **/
swif_status_t   swif_decoder_release (swif_decoder_t*        dec);


/**
 * Set the various callback functions for this decoder.
 * All the callback functions require an opaque context parameter, that
 * must be initialized accordingly by the application, since it is
 * application specific.
 *
 * @param dec   context (i.e., pointer to decoder structure).
 * @param source_symbol_removed_from_linear_system_callback
 *              (IN) Pointer to the function, within the application, that
 *              needs to be called each time a source symbol is removed from
 *              the left side of the linear system.
 *              This callback is called each time the linear system slides
 *              to the right and an old source symbol needs to be removed
 *              on the left. This function does not return anything.
 * @param decodable_source_symbol_callback
 *              (IN) Pointer to the function, within the application, that
 *              needs to be called each time a source symbol is decodable.
 *              What it does is application-dependent, but it MUST return
 *              either a pointer to a data buffer, left uninitialized, of
 *              the appropriate size, or NULL if the application prefers to
 *              let the codec allocate the buffer.
 *              In any case the codec is responsible for storing the actual
 *              symbol value within the data buffer. Also, no matter
 *              whether the data buffer is allocated by the application or
 *              the codec, it is the responsibility of the application to
 *              free this buffer when needed, once decoding is over (but
 *              not before since the codec does not keep any internal copy).
 * @param decoded_source_symbol_callback
 *              (IN) Pointer to the function, within the application, that
 *              needs to be called each time a source symbol is decodable and
 *              all computations performed (i.e., the buffer does contain the
 *              symbol value).
 *              This callback is called in a second time, when the newly
 *              decodable source symbol is actually decoded and ready,
 *              i.e., when all the computations (like XOR and GF(2**8)
 *              operations) have been performed. In any case, it is the
 *              responsibility of the application to free this buffer when
 *              needed, once decoding is over (but not before since the
 *              codec does not keep any internal copy). This function does
 *              not return anything.
 * @param context_4_callback
 *              (IN) Pointer to the application-specific context that will be
 *              passed to the callback function (if any). This context is not
 *              interpreted by this function.
 * @return
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
                void*        context_4_callback);


/**
 * This function sets one or more FEC codec specific parameters,
 *        using a type/length/value approach for maximum flexibility.
 *
 * @param dec           context (i.e., pointer to decoder structure).
 * @param type          (IN) Type of parameter.
 * @param length        (IN) length of the pointed value.
 * @param value         (IN) Pointer to the value. The exact type of
 *                      the object pointed is FEC codec specific.
 * @return
 */
swif_status_t   swif_decoder_set_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);


/**
 * This function gets one or more FEC codec specific parameters,
 * using a type/length/value approach for maximum flexibility.
 *
 * @param dec           context (i.e., pointer to decoder structure).
 * @param type          (IN) Type of parameter.
 * @param length        (IN) length of the pointed value.
 * @param value         (IN/OUT) Pointer to the value. The exact type of
 *                      the object pointed is FEC codec specific.
 *                      This function updates the value object
 *                      accordingly. The caller, who knows the FEC codec,
 *                      is responsible to allocate the appropriate
 *                      object buffer.
 * @return
 */
swif_status_t   swif_decoder_get_parameters  (
                                swif_decoder_t* dec,
                                uint32_t        type,
                                uint32_t        length,
                                void*           value);


/**
 * List here the FEC codec specific control parameters.
 */
enum {
        swif_DECODER_GET_PARAM_DECODER_STATISTICS = 1,
        swif_DECODER_SET_PARAM_RLC_DENSITY_THRESHOLD
};


/**
 * Submit a received source symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 *
 * This function usually returns SWIF_STATUS_OK, regardless of whether
 * this new symbol enabled the decoding of one or several source symbols,
 * or SWIF_STATUS_ERROR. It cannot return SWIF_STATUS_FAILURE.
 *
 * @param dec   context (i.e., pointer to decoder structure).
 * @param new_symbol_buf
 *              (IN) Pointer to the new source symbol now available (i.e.
 *              a new symbol received by the application, or a decoded
 *              symbol in case of a recursive call if it makes sense).
 * @param new_symbol_esi
 *              (IN) encoding symbol ID of the new source symbol.
 * @return
 */
swif_status_t   swif_decoder_decode_with_new_source_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_symbol_esi);


/**
 * Submit a received repair symbol and try to progress in the decoding.
 * For each decoded source symbol (if any), the application is informed
 * through the dedicated callback functions.
 *
 * This function requires that the application has previously initialized
 * the coding window and coding coefficients appropriately. The application
 * keeps a full control of the repair symbol buffer, i.e., the application
 * is in charge of freeing this buffer as soon as it believes appropriate
 * (a copy is kept by the codec). This is motivated by the fact that a
 * repair symbol may be part of a larger buffer (e.g., if there are
 * several repair symbols per packet, or because of a packet header): only
 * the application knows when the buffer can be safely freed.
 *
 * This function usually returns SWIF_STATUS_OK, regardless of whether
 * this new symbol enabled the decoding of one or several source symbols,
 * or SWIF_STATUS_ERROR. It cannot return SWIF_STATUS_FAILURE.
 *
 * @param dec   context (i.e., pointer to decoder structure).
 * @param new_symbol_buf
 *              (IN) Pointer to the new repair symbol now available (i.e.
 *              a new symbol received by the application or a decoded
 *              symbol in case of a recursive call if it makes sense).
 * @return
 */
swif_status_t   swif_decoder_decode_with_new_repair_symbol (
                                swif_decoder_t* dec,
                                void* const     new_symbol_buf,
                                esi_t           new_src_symbol_esi);


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
 *
 * @return
 */
swif_status_t   swif_encoder_reset_coding_window (swif_encoder_t*  enc);

swif_status_t   swif_decoder_reset_coding_window (swif_decoder_t*  dec);


/**
 * Add this source symbol to the coding window.
 * Encoder:     add a source symbol to the coding window.
 * Decoder:     add a source symbol to the coding window under preparation.
 *
 * @param new_src_symbol_buf    (encoder only) pointer to a buffer
 *              containing the source symbol. The application MUST NOT
 *              free nor modify this buffer as long as the source symbol
 *              is in the coding window.
 * @param new_src_symbol_esi    ESI of the source symbol to add.
 * @return
 */
swif_status_t   swif_encoder_add_source_symbol_to_coding_window (
                                swif_encoder_t* enc,
                                void*           new_src_symbol_buf,
                                esi_t           new_src_symbol_esi);

swif_status_t   swif_decoder_add_source_symbol_to_coding_window (
                                swif_decoder_t* dec,
                                esi_t           new_src_symbol_esi);


/**
 * Remove this source symbol from the coding window.
 *
 * Encoder:     remove a source symbol from the encoding window, e.g.
 *              because the application knows that a source symbol has
 *              been acknowledged by the peer (if applicable). Note that
 *              the left side of the sliding window is automatically
 *              managed by the codec and no action is needed from the
 *              application. If needed a callback is available to inform
 *              the application that a source symbol has been removed).
 * Decoder:     remove a source symbol from the coding window under
 *              preparation.
 *
 * @param old_src_symbol_esi    ESI of the source symbol to remove from
 *              the coding window.
 * @return
 */
swif_status_t   swif_encoder_remove_source_symbol_from_coding_window (
                                swif_encoder_t* enc,
                                esi_t           old_src_symbol_esi);

swif_status_t   swif_decoder_remove_source_symbol_from_coding_window (
                                swif_decoder_t* dec,
                                esi_t           old_src_symbol_esi);


/**
 * Get information on the current coding window at the encoder.
 * This function stores the ESI of the first source symbol and
 * last source symbol in the coding window, as well as the number
 * of symbols. In theory the application should be able to recover
 * the information (it knows when new symbols are added and old
 * symbols removed), but it's easier to let the SWiF Codec care
 * about it. The number of source symbols is also returned.
 * In situations where there's no gap (i.e., when
 * swif_encoder_remove_source_symbol_from_coding_window() has not
 * been used), nss can also be calculated with first/last. However
 * it is more convenient to use nss directly (in particular in case
 * of wraping to zero of either first or last).
 *
 * @param enc
 * @param first		(in/out) pointer to ESI of the first source
 *			symbol in the coding window (inclusive)
 * @param last		(in/out) pointer to ESI of the last source
 *			symbol in the coding window (inclusive)
 * @param nss		(in/out) pointer to number of source symbols
 *			in the coding window
 * @return
 */
swif_status_t	swif_encoder_get_coding_window_information (
				swif_encoder_t*	enc,
				esi_t*		first,
				esi_t*		last,
				uint32_t*	nss);


/*******************************************************************************
 * Coding Coefficients Functions at an Encoder and Decoder
 */


/**
 * The following functions enable an encoder (resp. decoder) to
 * initialize the set of coefficients to be used for encoding
 * or associated to a received repair symbol.
 *
 * Encoder: calling one of them MUST be done before calling
 *              build_repair_symbol().
 * Decoder: calling one of them MUST be done before calling
 *              decode_with_new_repair_symbol().
 */

/**
 * Encoder: this function specifies the coding coefficients chosen by
 *          the application if this is the way the codec works.
 * Decoder: communicate with this function the coding coefficients
 *          associated to a repair symbol and carried in the packet
 *          header.
 *
 * @param coding_coefs_tab
 *              (IN) table of coding coefficients associated to each of
 *              the source symbols currently in the encoding window.
 *              The size (number of bits) of each coefficient depends on
 *              the FEC Scheme. The allocation and release of this table
 *              is under the responsibility of the application.
 * @param nb_coefs_in_tab
 *              (IN) number of entries (i.e., coefficients) in the table.
 * @return
 */
swif_status_t   swif_encoder_set_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void*           coding_coefs_tab,
                                uint32_t        nb_coefs_in_tab);

swif_status_t   swif_decoder_set_coding_coefs_tab (
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
 *
 * @param key   (IN) Value that can be used as a seed in case of a PRNG
 *              for instance, or by a specific coding coefficients
 *              function. Set to 0 if not required by a codec.
 * @param add_param
 *              (IN) an opaque 32-bit integer that contains a codec
 *              specific parameter if needed. Set to 0 if not used.
 * @return
 */
swif_status_t   swif_encoder_generate_coding_coefs (
                                swif_encoder_t* enc,
                                uint32_t        key,
                                uint32_t        add_param);

swif_status_t   swif_decoder_generate_coding_coefs (
                                swif_decoder_t* dec,
                                uint32_t        key,
                                uint32_t        add_param);


/**
 * This function enables the application to retrieve the set of coding
 * coefficients generated and used by build_repair_symbol(). This is
 * useful when the choice of coefficients is performed by the codec in
 * an autonomous manner but needs to be sent in the repair packet header.
 * This function is only used by an encoder.
 *
 * @param coding_coefs_tab
 *              (OUT) pointer to a table of coding coefficients.
 *              The size (number of bits) of each coefficient depends on
 *              the FEC scheme. Upon return of this function, this table
 *              is allocated and filled with coefficient values. The
 *              release of this table is under the responsibility of the
 *              application.
 * @param nb_coefs_in_tab
 *              (IN/OUT) pointer to the number of entries (i.e.,
 *              coefficients) in the table.
 *              Upon calling this function, this number must be zero.
 *              Upon return of this function this variable is initialized
 *              with the actual number of entries in the coeffs_tab[].
 * @return
 */
swif_status_t   swif_encoder_get_coding_coefs_tab (
                                swif_encoder_t* enc,
                                void**          coding_coefs_tab,
                                uint32_t*       nb_coefs_in_tab);

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __SWIF_API_H__ */
