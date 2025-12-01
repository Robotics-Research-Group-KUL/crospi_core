#ifndef LOGGERHANDLER_HPP
#define LOGGERHANDLER_HPP

#include "crospi_utils/etasl_error.hpp"
#include <fmt/format.h>
#include <iostream>
#include <memory>
namespace etasl {

enum Level {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class LogHandler {
public:
    typedef std::shared_ptr<LogHandler> SharedPtr;

    /**
     * log the given string
     */
    virtual void log(Level L, const std::string& s) = 0;

    template <typename... T>
    void log(Level L,fmt::format_string<T...> fmt, T&&... args)
    {
        const auto& vargs = fmt::make_format_args(args...);
        log(L,fmt::vformat(fmt, vargs) );
    }

    virtual ~LogHandler() {};
};
class DefaultLogHandler : public LogHandler {
public:
    /**
     * log the given string
     */
    virtual void log(Level L, const std::string& s) override
    {
        // std::cout << s << std::endl;
    }
};

} // namespace etasl

#endif