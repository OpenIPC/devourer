#ifdef WITH_LIBLC

/* A temporary adapter based on liblc - no longer used */
#include "swif_full_symbol_lc.c"

#else /* WITH_LIBLC */

/* A new implementation based on swif_symbol */
#include "swif_full_symbol_impl.c"

#endif /* WITH_LIBLC */
