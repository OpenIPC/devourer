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

#include "swif_includes.h"

static void tinymt32_next_state (tinymt32_t * s);
static uint32_t tinymt32_temper (tinymt32_t * s);

/**
 * Parameter set to use for this IETF specification. Don't change.
 * This parameter set is the first entry of the precalculated
 * parameter sets in file tinymt32dc.0.1048576.txt, by Kenji
 * Rikitake, available at:
 *    https://github.com/jj1bdx/tinymtdc-longbatch/blob/master/
 *    tinymt32dc/tinymt32dc.0.1048576.txt
 * It is also the parameter set used:
 *    Rikitake, K., "TinyMT Pseudo Random Number Generator for
 *    Erlang", ACM 11th SIGPLAN Erlang Workshop (Erlang'12),
 *    September, 2012.
 */
const uint32_t  TINYMT32_MAT1_PARAM = UINT32_C(0x8f7011ee);
const uint32_t  TINYMT32_MAT2_PARAM = UINT32_C(0xfc78ff1f);
const uint32_t  TINYMT32_TMAT_PARAM = UINT32_C(0x3793fdff);

/**
 * This function initializes the internal state array with a
 * 32-bit unsigned integer seed.
 * @param s     pointer to tinymt internal state.
 * @param seed  a 32-bit unsigned integer used as a seed.
 */
void tinymt32_init (tinymt32_t * s, uint32_t seed)
{
    const uint32_t    MIN_LOOP = 8;
    const uint32_t    PRE_LOOP = 8;
    s->status[0] = seed;
    s->status[1] = s->mat1 = TINYMT32_MAT1_PARAM;
    s->status[2] = s->mat2 = TINYMT32_MAT2_PARAM;
    s->status[3] = s->tmat = TINYMT32_TMAT_PARAM;
    for (int i = 1; i < MIN_LOOP; i++) {
        s->status[i & 3] ^= i + UINT32_C(1812433253)
            * (s->status[(i - 1) & 3]
               ^ (s->status[(i - 1) & 3] >> 30));
    }
    /*
     * NB: the parameter set of this specification warrants
     * that none of the possible 2^^32 seeds leads to an
     * all-zero 127-bit internal state. Therefore, the
     * period_certification() function of the original
     * TinyMT32 source code has been safely removed. If
     * another parameter set is used, this function will
     * have to be re-introduced here.
     */
    for (int i = 0; i < PRE_LOOP; i++) {
        tinymt32_next_state(s);
    }
}

/**
 * This function outputs a 32-bit unsigned integer from
 * the internal state.
 * @param s	pointer to tinymt internal state.
 * @return	32-bit unsigned integer r (0 <= r < 2^32).
 */
uint32_t tinymt32_generate_uint32 (tinymt32_t * s)
{
    tinymt32_next_state(s);
    return tinymt32_temper(s);
}

/**
 * Internal tinymt32 constants and functions.
 * Users should not call these functions directly.
 */
const uint32_t	TINYMT32_SH0 = 1;
const uint32_t	TINYMT32_SH1 = 10;
const uint32_t	TINYMT32_SH8 = 8;
const uint32_t	TINYMT32_MASK = UINT32_C(0x7fffffff);

/**
 * This function changes the internal state of tinymt32.
 * @param s	pointer to tinymt internal state.
 */
static void tinymt32_next_state (tinymt32_t * s)
{
    uint32_t x;
    uint32_t y;

    y = s->status[3];
    x = (s->status[0] & TINYMT32_MASK)
        ^ s->status[1]
        ^ s->status[2];
    x ^= (x << TINYMT32_SH0);
    y ^= (y >> TINYMT32_SH0) ^ x;
    s->status[0] = s->status[1];
    s->status[1] = s->status[2];
    s->status[2] = x ^ (y << TINYMT32_SH1);
    s->status[3] = y;
    /*
     * The if (y & 1) {...} block below replaces:
     *     s->status[1] ^= -((int32_t)(y & 1)) & s->mat1;
     *     s->status[2] ^= -((int32_t)(y & 1)) & s->mat2;
     * The adopted code is equivalent to the original code
     * but does not depend on the representation of negative
     * integers by 2's complements. It is therefore more
     * portable, but includes an if-branch which may slow
     * down the generation speed.
     */
    if (y & 1) {
         s->status[1] ^= s->mat1;
         s->status[2] ^= s->mat2;
     }
}

/**
 * This function outputs a 32-bit unsigned integer from
 * the internal state.
 * @param s	pointer to tinymt internal state.
 * @return	32-bit unsigned pseudo-random number.
 */
static uint32_t tinymt32_temper (tinymt32_t * s)
{
    uint32_t t0, t1;
    t0 = s->status[3];
    t1 = s->status[0] + (s->status[2] >> TINYMT32_SH8);
    t0 ^= t1;
    t0 ^= -((int32_t)(t1 & 1)) & s->tmat;
    return t0;
}

