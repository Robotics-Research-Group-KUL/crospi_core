// Copyright Daniel Wallin, David Abrahams 2005.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_VOID_050329_HPP
#define CROSPI_BOOST_PARAMETER_VOID_050329_HPP

namespace crospi_boost { namespace parameter {

    // A placemarker for "no argument passed."
    // MAINTAINER NOTE: Do not make this into a metafunction
    struct void_
    {
    };
}} // namespace crospi_boost::parameter

namespace crospi_boost { namespace parameter { namespace aux {

    inline ::crospi_boost::parameter::void_& void_reference()
    {
        static ::crospi_boost::parameter::void_ instance;
        return instance;
    }
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/config/workaround.hpp>

#if CROSPI_BOOST_WORKAROUND(__SUNPRO_CC, CROSPI_BOOST_TESTED_AT(0x580))

namespace crospi_boost { namespace parameter { namespace aux {

    typedef void* voidstar;
}}} // namespace crospi_boost::parameter::aux

#endif
#endif  // include guard

