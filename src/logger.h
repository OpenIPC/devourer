#ifndef LOGGER_H
#define LOGGER_H

#include <memory>
#include <string>

#include <string_view>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

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

    #define RESET "\033[0m"
    #define RED "\033[31m"
    #define GREEN "\033[32m"
    #define YELLOW "\033[33m"

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
    printf(YELLOW "<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                       \
    printf("\n" RESET)
    #define DEVOURER_LOGE(...)                \
    printf(RED "<%s>", DEVOURER_LOG_TAG); \
    printf(__VA_ARGS__);                    \
    printf("\n" RESET)
#endif

template<typename T>
void format_helper(std::ostringstream& oss,
                   std::string_view& str, const T& value)
{
    std::size_t openBracket = str.find('{');
    if (openBracket == std::string::npos) { return; }
    std::size_t closeBracket = str.find('}', openBracket + 1);
    if (closeBracket == std::string::npos) { return; }
    oss << str.substr(0, openBracket) << value;
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
    template<typename... Args>
    void debug(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        DEVOURER_LOGD(txt.c_str());
    }

    template<typename... Args>
    void info(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        DEVOURER_LOGI(txt.c_str());
    }

    template<typename... Args>
    void warn(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        DEVOURER_LOGW(txt.c_str());
    }

    template<typename... Args>
    void error(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        DEVOURER_LOGE(txt.c_str());
    }
};

using Logger_t = std::shared_ptr<Logger>;

#endif /* LOGGER_H */
