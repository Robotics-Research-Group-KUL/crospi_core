//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Erwin Aertbeliën
//  email: <erwin.aertbelien@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#ifndef ETASL_ERROR_HPP_32DA23
#define ETASL_ERROR_HPP_32DA23

#include <string>
#include <stdexcept>
#include <fmt/format.h>
#include <source_location>

namespace etasl {
class etasl_error : public std::exception {
    int code;
    std::string descript;

public:
    enum ErrorCode {
        SUCCESS = 0,
        FAILED_TO_LOAD,
        LUA_PARSER_ERROR,
        INITIALIZER_ERROR,
        FAILED_TO_CREATE_SOLVER,
        SOLVER_FAILED_DURING_RUN,
        UNKNOWN_SOLVER_REQUESTED,
        UNKNOWN_SOLVER_PARAMETER,
        SOLVER_PARAMETER_NOT_SPECIFIED,
        MISSING_PARAMETER,
        WRONG_TYPE_OF_PARAMETER,
        UNKNOWN_PARAMETER,
        PARAMETER_CHECK_FAILED,
        STRING_INTERPOLATION_ERROR,
        XML_PARSE_ERROR,
        JSON_PARSE_ERROR,
        INITIALIZER_PREPARE_FAILED,
        INITIALIZER_FULL_NOT_SUPPORTED,
        INITIALIZER_SOLVER_ERROR,
        INITIALIZER_NOT_CONVERGED,
        NOT_REGISTERED,
        PARAMETER_ERROR,
        INPUTHANDLER_INCONSISTENT_MESSAGE,
        OUTPUTHANDLER_UKNOWN_OUTPUT,
        SCHEMA_REFERENCE_NOT_FOUND
    };

    /*etasl_error(int _code, const std::string _descript)
        : code(_code)
        , descript(_descript)
    {
    }*/

    template <typename... T>
    etasl_error(ErrorCode _code, fmt::format_string<T...> fmt, T&&... args)
    : descript( fmt::vformat(fmt, fmt::make_format_args(args...)) )
    , code(_code)
    {
    }


    virtual const char* what() const noexcept override
    {
        return descript.c_str();
    }
    int getCode() const
    {
        return code;
    }
};

} // namespace etasl
#endif