#ifndef LOGGER_H
#define LOGGER_H

#include <memory>
#include <string>

#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <type_traits>

#define ushort uint16_t
#define DEVOURER_LOG_TAG "devourer"

#ifdef __ANDROID__
    #include <android/log.h>

    #define DEVOURER_LOGV(...) __android_log_write(ANDROID_LOG_VERBOSE, DEVOURER_LOG_TAG, __VA_ARGS__)
    #define DEVOURER_LOGD(...) __android_log_write(ANDROID_LOG_DEBUG, DEVOURER_LOG_TAG, __VA_ARGS__)
    #define DEVOURER_LOGI(...) __android_log_write(ANDROID_LOG_INFO, DEVOURER_LOG_TAG, __VA_ARGS__)
    #define DEVOURER_LOGW(...) __android_log_write(ANDROID_LOG_WARN, DEVOURER_LOG_TAG, __VA_ARGS__)
    #define DEVOURER_LOGE(...) __android_log_write(ANDROID_LOG_ERROR, DEVOURER_LOG_TAG, __VA_ARGS__)
#else
    #include <cstdio>

    #define DEVOURER_LOGV(...)            \
    printf("<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                \
    printf("\n")
    #define DEVOURER_LOGD(...)            \
    printf("<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                \
    printf("\n")
    #define DEVOURER_LOGI(...)            \
    printf("<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                \
    printf("\n")
    #define DEVOURER_LOGW(...)                   \
    printf("<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                       \
    printf("\n")
    #define DEVOURER_LOGE(...)                \
    printf("<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                    \
    printf("\n")
#endif

/* Widen byte-sized integers before streaming: `oss << uint8_t` inserts a raw
 * char, which puts non-UTF-8 bytes into the log (register values like 0xF0)
 * and breaks text consumers such as tests/regress.py. Plain `char` is left
 * alone — it is a distinct type and means "character" at our call sites. */
template<typename T>
decltype(auto) widen_for_stream(const T& value)
{
    if constexpr (std::is_same_v<T, unsigned char>)
        return static_cast<unsigned>(value);
    else if constexpr (std::is_same_v<T, signed char>)
        return static_cast<int>(value);
    else
        return (value);
}

template<typename T>
void format_helper(std::ostringstream& oss,
                   std::string_view& str, const T& value)
{
    std::size_t openBracket = str.find('{');
    if (openBracket == std::string::npos) { return; }
    std::size_t closeBracket = str.find('}', openBracket + 1);
    if (closeBracket == std::string::npos) { return; }

    oss << str.substr(0, openBracket);

    /* Parse a minimal subset of std::format spec inside {...}: optional
     * ':' then optional '0' fill, optional width digits, optional 'x'/'X'
     * for hex. Sufficient for our uses; falls back to default formatting
     * if no spec given. */
    std::string_view spec = str.substr(openBracket + 1, closeBracket - openBracket - 1);
    if (auto colon = spec.find(':'); colon != std::string_view::npos) {
        spec.remove_prefix(colon + 1);
    } else {
        spec = {};
    }

    if (!spec.empty()) {
        std::ios_base::fmtflags saved = oss.flags();
        char saved_fill = oss.fill();
        bool zero_pad = false;
        if (!spec.empty() && spec.front() == '0') {
            zero_pad = true;
            spec.remove_prefix(1);
        }
        int width = 0;
        while (!spec.empty() && std::isdigit(static_cast<unsigned char>(spec.front()))) {
            width = width * 10 + (spec.front() - '0');
            spec.remove_prefix(1);
        }
        bool hex = false, hex_upper = false;
        if (!spec.empty() && (spec.front() == 'x' || spec.front() == 'X')) {
            hex = true;
            hex_upper = spec.front() == 'X';
        }
        if (hex) oss << std::hex;
        if (hex_upper) oss << std::uppercase;
        if (zero_pad) oss << std::setfill('0');
        if (width > 0) oss << std::setw(width);
        oss << widen_for_stream(value);
        oss.flags(saved);
        oss.fill(saved_fill);
    } else {
        oss << widen_for_stream(value);
    }

    str = str.substr(closeBracket + 1);
}

template<typename... Targs>
std::string format(std::string_view str, Targs...args)
{
    std::ostringstream oss;
    (format_helper(oss, str, args),...);
    oss << str;
    return oss.str();
}

template <typename... Args>
using format_string_t = std::string_view;

class Logger {
public:
    /* Verbosity threshold. A message at level L is emitted only when
     * _level <= L, so Debug shows everything and Silent shows nothing. Defaults
     * to Debug, so existing consumers' output is unchanged; a caller that wants
     * a clean stdout (e.g. sense's live display) can quiet the library
     * with set_level(Logger::Level::Warn). */
    enum class Level { Debug, Info, Warn, Error, Silent };
    void set_level(Level l) { _level = l; }
    Level level() const { return _level; }

    template<typename... Args>
    void debug(format_string_t<Args...> fmt, Args &&...args) {
#if !defined(NDEBUG)
        if (_level > Level::Debug) return;
        std::string txt = format(fmt, args...);
        DEVOURER_LOGD(txt.c_str());
#endif
    }

    template<typename... Args>
    void info(format_string_t<Args...> fmt, Args &&...args) {
        if (_level > Level::Info) return;
        std::string txt = format(fmt, args...);
        DEVOURER_LOGI(txt.c_str());
    }

    template<typename... Args>
    void warn(format_string_t<Args...> fmt, Args &&...args) {
        if (_level > Level::Warn) return;
        std::string txt = format(fmt, args...);
        DEVOURER_LOGW(txt.c_str());
    }

    template<typename... Args>
    void error(format_string_t<Args...> fmt, Args &&...args) {
        if (_level > Level::Error) return;
        std::string txt = format(fmt, args...);
        DEVOURER_LOGE(txt.c_str());
    }

private:
    Level _level = Level::Debug;
};

using Logger_t = std::shared_ptr<Logger>;

#endif /* LOGGER_H */
