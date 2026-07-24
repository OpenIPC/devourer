// ---------------------------------------------------------------------------
//  compat/android/log.h — drop-in shim for non-Android (native Windows/Linux)
//  builds of the libusb driver.
//
//  The driver's diagnostics call __android_log_print(prio, tag, fmt, ...) and
//  #include <android/log.h> directly. On Android the NDK provides the real
//  header. On a host build this directory is added to the include path ONLY
//  when NOT ANDROID (see CMakeLists), so <android/log.h> resolves here and the
//  same source compiles unchanged — diagnostics go to stderr instead of logcat.
// ---------------------------------------------------------------------------
#ifndef APFPV_COMPAT_ANDROID_LOG_H
#define APFPV_COMPAT_ANDROID_LOG_H

#include <cstdarg>
#include <cstdio>

#ifndef ANDROID_LOG_VERBOSE
typedef enum android_LogPriority {
    ANDROID_LOG_UNKNOWN = 0,
    ANDROID_LOG_DEFAULT,
    ANDROID_LOG_VERBOSE,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
    ANDROID_LOG_FATAL,
    ANDROID_LOG_SILENT,
} android_LogPriority;
#endif

// printf-style sink. `inline` (not static) so all TUs share one definition and
// there's no "defined but not used" warning when a TU includes the header but
// doesn't log.
inline int __android_log_print(int prio, const char *tag, const char *fmt, ...) {
    (void)prio;
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "[%s] ", tag ? tag : "?");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
    return 0;
}

#endif /* APFPV_COMPAT_ANDROID_LOG_H */
