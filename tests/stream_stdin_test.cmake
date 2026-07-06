# ctest driver for StreamStdinSelftest — see examples/common/stream_stdin_selftest.cpp.
#
# Pipes the self-test's --gen output (the canonical <u32_le len><PSDU> stream,
# which embeds 0x1A / 0x0D / 0x0A) straight into a second invocation that reads
# it back through the binary-stdin path and round-trips it. The anonymous pipe
# CMake sets up is byte-exact; the only thing that can corrupt the stream is a
# process putting its own stdio into text mode. So:
#
#   * correct (_WIN32-gated) binary mode  -> reader prints "...OK", exit 0, PASS
#   * regressed (_MSC_VER gate on mingw)  -> 0x1A read as EOF / CRLF mangled
#                                            -> reader exits non-zero, FAIL
#
# Off Windows there is no text mode, so the test always passes there; its bite
# is on the Windows (MSVC) and mingw CI jobs. Invoked with -DSELFTEST_EXE=<path>.

if(NOT SELFTEST_EXE)
  message(FATAL_ERROR "SELFTEST_EXE not set")
endif()

execute_process(
  COMMAND "${SELFTEST_EXE}" --gen
  COMMAND "${SELFTEST_EXE}"
  OUTPUT_VARIABLE check_out
  ERROR_VARIABLE check_err
  RESULTS_VARIABLE all_rc)

# RESULTS_VARIABLE is a ;-list of each pipeline stage's exit code.
foreach(rc IN LISTS all_rc)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR
      "StreamStdinSelftest pipeline stage exited ${rc} — binary stdin/stdout is "
      "broken (a 0x1A or CRLF byte corrupted the length-prefixed stream). Check "
      "set_stdin_binary()/set_stdout_binary() in examples/common/stream_stdin.h.\n"
      "stdout:\n${check_out}\nstderr:\n${check_err}")
  endif()
endforeach()

if(NOT check_out MATCHES "records=5 bytes=20 OK")
  message(FATAL_ERROR
    "StreamStdinSelftest produced unexpected output:\n${check_out}\n"
    "stderr:\n${check_err}")
endif()

message(STATUS "stream_stdin round-trip OK: ${check_out}")
