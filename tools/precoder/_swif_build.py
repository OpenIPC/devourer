"""cffi build script for the vendored swif-codec library.

Run with `python _swif_build.py` to produce `_swif_ext*.so` next to this
file. The generated module is `_swif_ext`; `stream_fec_rlc.py` does
`from _swif_ext import ffi, lib`.

The cffi `cdef` section declares only the subset of the swif API we
actually use from Python — the RLC-specific helpers plus the generic
encoder/decoder type-erased pointer plumbing. We skip the
TLV-style parameter setter/getter machinery and reach down to the codec's
direct-call helpers because the cdef-able C subset is smaller that way.
"""

from __future__ import annotations

import glob
import os
import sys

from cffi import FFI

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "vendor", "swif-codec", "src")

# Upstream uses a C-amalgamation pattern: swif_api.c does
# `#include "swif_rlc_api.c"` which in turn does
# `#include "swif_full_symbol_impl.c"`. Compiling swif_api.c alone yields
# the entire codec; compiling it + swif_rlc_api.c + swif_full_symbol.c
# triggers multiple-definition linker errors because all three .o files
# define `full_symbol_scale`, `full_symbol_add`, etc.
#
# So our build list is the four LEAF translation units the amalgamation
# does NOT pull in by `#include`:
_LIB_SOURCES = [
    "swif_api.c",                # pulls in swif_rlc_api.c + swif_full_symbol_impl.c
    "swif_coding_coefficients.c",
    "swif_prng.c",                # tinymt32_init / tinymt32_generate_uint32
    "swif_symbol.c",              # gf256_mul / gf256_inv / symbol_add_scaled
]
C_FILES = [os.path.join(SRC, name) for name in _LIB_SOURCES]
for path in C_FILES:
    if not os.path.isfile(path):
        raise SystemExit(
            f"_swif_build.py: vendored source {path} missing — has the "
            f"swif-codec snapshot been updated?"
        )

ffi = FFI()

# Public surface — keep this slim so the cdef stays auditable. Everything we
# call from Python goes here; helper code that's only invoked internally
# (swif_full_symbol_impl, the GF(2^8) tables) stays inside the C TU.
ffi.cdef(r"""
    typedef enum {
        SWIF_STATUS_OK = 0,
        SWIF_STATUS_FAILURE,
        SWIF_STATUS_ERROR
    } swif_status_t;

    typedef enum {
        SWIF_CODEPOINT_NULL = 0,
        SWIF_CODEPOINT_RLC_GF_256_FULL_DENSITY_CODEC
    } swif_codepoint_t;

    typedef uint32_t esi_t;

    /* The generic encoder/decoder structs carry function-pointer tables.
     * We only expose the pointer; all calls go through the RLC-specific
     * wrappers below. */
    typedef struct swif_encoder { ...; } swif_encoder_t;
    typedef struct swif_decoder { ...; } swif_decoder_t;

    /* === RLC encoder === */
    swif_encoder_t* swif_rlc_encoder_create(
        swif_codepoint_t codepoint,
        uint32_t verbosity,
        uint32_t symbol_size,
        uint32_t max_coding_window_size);

    swif_status_t swif_rlc_encoder_release(swif_encoder_t* enc);

    swif_status_t swif_rlc_encoder_add_source_symbol_to_coding_window(
        swif_encoder_t* enc, void* new_src_symbol_buf, esi_t esi);

    swif_status_t swif_rlc_encoder_remove_source_symbol_from_coding_window(
        swif_encoder_t* enc, esi_t old_esi);

    swif_status_t swif_rlc_encoder_get_coding_window_information(
        swif_encoder_t* enc, esi_t* first, esi_t* last, uint32_t* nss);

    swif_status_t swif_encoder_generate_coding_coefs(
        swif_encoder_t* enc, uint32_t key, uint32_t dt);

    swif_status_t swif_rlc_build_repair_symbol(
        swif_encoder_t* enc, void** new_buf);

    /* === RLC decoder === */
    swif_decoder_t* swif_rlc_decoder_create(
        swif_codepoint_t codepoint,
        uint32_t verbosity,
        uint32_t symbol_size,
        uint32_t max_coding_window_size,
        uint32_t max_linear_system_size);

    swif_status_t swif_rlc_decoder_release(swif_decoder_t* dec);

    /* The decoder reports decoded source symbols through this callback
     * pair (set once, the codec holds the function pointers and a context).
     * Python provides cffi `@ffi.callback`-decorated handlers.
     *
     * decodable_source_symbol_callback returns a void* — the application's
     * destination buffer where the codec will write the decoded bytes.
     * The codec calls decoded_source_symbol_callback once those bytes are
     * in place; the application then takes ownership.
     */
    swif_status_t swif_rlc_decoder_set_callback_functions(
        swif_decoder_t* dec,
        void (*source_symbol_removed_from_linear_system_callback)(
            void* context, esi_t old_symbol_esi),
        void* (*decodable_source_symbol_callback)(
            void* context, esi_t esi),
        void* (*decoded_source_symbol_callback)(
            void* context, void* new_symbol_buf, esi_t esi),
        void* context_4_callback);

    swif_status_t swif_rlc_decoder_decode_with_new_source_symbol(
        swif_decoder_t* dec, void* const new_symbol_buf, esi_t esi);

    swif_status_t swif_rlc_decoder_decode_with_new_repair_symbol(
        swif_decoder_t* dec, void* const new_symbol_buf, esi_t esi);

    swif_status_t swif_rlc_decoder_reset_coding_window(swif_decoder_t* dec);

    swif_status_t swif_rlc_decoder_add_source_symbol_to_coding_window(
        swif_decoder_t* dec, esi_t esi);

    swif_status_t swif_decoder_generate_coding_coefs(
        swif_decoder_t* dec, uint32_t key, uint32_t dt);
""")

ffi.set_source(
    "_swif_ext",
    r"""
        #include <stdint.h>
        #include "swif_rlc_api.h"
    """,
    sources=C_FILES,
    include_dirs=[SRC],
    extra_compile_args=["-std=c99", "-O2", "-Wno-unused-parameter",
                        "-Wno-unused-variable", "-Wno-unused-function",
                        "-Wno-pointer-sign", "-Wno-incompatible-pointer-types",
                        "-Wformat", "-Wno-error=format-security"],
)


if __name__ == "__main__":
    # cffi's ffi.compile() funnels through setuptools' distutils which on
    # modern setuptools mishandles absolute paths in `sources=` (the link
    # step looks for objects under the build tmpdir, but the compile step
    # placed them under their absolute path). We bypass setuptools by
    # emitting the wrapper C and driving gcc directly.
    import subprocess
    import sysconfig

    # 1. Emit the cffi-generated wrapper C source.
    c_file = os.path.join(HERE, "_swif_ext.c")
    ffi.emit_c_code(c_file)

    # 2. Compile all C files (wrapper + vendored sources) and link the .so.
    py_inc = sysconfig.get_path("include")
    ext_suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
    so_path = os.path.join(HERE, f"_swif_ext{ext_suffix}")
    cmd = [
        "gcc", "-fPIC", "-shared",
        "-std=c99", "-O2",
        "-Wno-unused-parameter", "-Wno-unused-variable",
        "-Wno-unused-function", "-Wno-pointer-sign",
        "-Wno-incompatible-pointer-types",
        "-Wformat", "-Wno-error=format-security",
        f"-I{py_inc}", f"-I{SRC}",
        c_file, *C_FILES,
        "-o", so_path,
    ]
    print(" ".join(cmd), file=sys.stderr)
    subprocess.check_call(cmd)
    print(f"_swif_ext built at {so_path}", file=sys.stderr)
