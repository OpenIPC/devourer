/**
 * SWiF Codec: an open-source sliding window FEC codec in C
 * https://github.com/irtf-nwcrg/swif-codec
 */

/**
 * Tiny Mersenne Twister, only 127 bit internal state.
 * Derived from the reference implementation version 1.1 (2015/04/24)
 * by Mutsuo Saito (Hiroshima University) and Makoto Matsumoto
 * (Hiroshima University).
 *
 * PRNG coming from RLC FEC Scheme and TinyMT PRNG Internet-Drafts:
 * https://datatracker.ietf.org/doc/draft-ietf-tsvwg-rlc-fec-scheme/
 * https://datatracker.ietf.org/doc/draft-roca-tsvwg-tinymt32/
 */


/**
 * tinymt32 internal state vector and parameters
 */
typedef struct {
    uint32_t status[4];
    uint32_t mat1;
    uint32_t mat2;
    uint32_t tmat;
} tinymt32_t;


/**
 * This function initializes the internal state array with a 32-bit
 * unsigned integer seed.
 * @param s pointer to tinymt internal state.
 * @param seed  a 32-bit unsigned integer used as a seed.
 */
void tinymt32_init     (tinymt32_t *	s,
			uint32_t	seed);


/**
 * This function outputs a 32-bit unsigned integer from
 * the internal state.
 * @param s	pointer to tinymt internal state.
 * @return	32-bit unsigned integer r (0 <= r < 2^32).
 */
uint32_t tinymt32_generate_uint32 (tinymt32_t * s);


/**
 * This function outputs a pseudo-random integer in [0 .. 15] range.
 * Used by RLC FEC Scheme:
 * https://datatracker.ietf.org/doc/draft-ietf-tsvwg-rlc-fec-scheme/
 *
 * @param s     pointer to tinymt internal state.
 * @return      unsigned integer between 0 and 15 inclusive.
 */
static inline uint32_t tinymt32_rand16(tinymt32_t *s)
{
    return (tinymt32_generate_uint32(s) & 0xF);
}


/**
 * This function outputs a pseudo-random integer in [0 .. 255] range.
 * Used by RLC FEC Scheme:
 * https://datatracker.ietf.org/doc/draft-ietf-tsvwg-rlc-fec-scheme/
 *
 * @param s     pointer to tinymt internal state.
 * @return      unsigned integer between 0 and 255 inclusive.
 */
static inline uint32_t tinymt32_rand256(tinymt32_t *s)
{
    return (tinymt32_generate_uint32(s) & 0xFF);
}

