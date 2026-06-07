
## "Full symbol" and "Full symbol set"

The concept of "full symbol" is a symbol  associated with the coefficients
of the source symbols (e.g. encoding vector), that specifies the linear
combination corresponding to the symbol.

Symbols can be added, combined, etc. They are mostly used in with
a "full symbol set" that performs Gaussian Elimination.

## Status

  * Interface has been defined for "full symbol"
  * Python wrappers are being implemented to test (for testing)
  * Initial implementation will be based on liblc (just to get it work)

## Interface

  * The interface is realized as an internal API for swif, intended for the
decoder. It is defined in  `swif_full_symbol.h`
  * The step-1 adaptation, using liblc, is in `swif_full_symbol_lc.c`

## Compilation

  * In order to actually compile with liblc you need to:
      * checkout `liblc` in the root of swif-codec
      (shortcut: `cd wrapper/python && make ensure-liblc`)
      * actually compile the liblc adapters, by including Makefile.liblc in the Makefile
      (shortcut: `ln -s Makefile.liblc Makefile.local`)

  * The test is in:
    `cd wrapper/python && make && python test_swif.py`

