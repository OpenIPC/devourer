#ifndef LOGGER_H
#define LOGGER_H

#include <memory>
#include <string>
#include <android/log.h>

#include <string_view>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>

#define ushort uint16_t
#define TAG "com.geehe.fpvue"

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

class logger {
public:
    template<typename... Args>
    void debug(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        __android_log_write(ANDROID_LOG_DEBUG, TAG, txt.c_str());
    }

    template<typename... Args>
    void info(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        __android_log_write(ANDROID_LOG_INFO, TAG, txt.c_str());
    }

    template<typename... Args>
    void warn(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        __android_log_write(ANDROID_LOG_WARN, TAG, txt.c_str());
    }

    template<typename... Args>
    void error(format_string_t<Args...> fmt, Args &&...args) {
        std::string txt = format(fmt, args...);
        __android_log_write(ANDROID_LOG_ERROR, TAG, txt.c_str());
    }
};

using Logger_t = std::shared_ptr<logger>;

#endif /* LOGGER_H */
