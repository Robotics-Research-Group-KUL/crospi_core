// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_TAGGED_ARGUMENT_FWD_HPP
#define CROSPI_BOOST_PARAMETER_TAGGED_ARGUMENT_FWD_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Arg>
    class tagged_argument;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Arg>
    struct tagged_argument_rref;
}}} // namespace crospi_boost::parameter::aux

#endif

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename TaggedArg>
    struct tagged_argument_list_of_1;
}}} // namespace crospi_boost::parameter::aux

#endif
#endif  // include guard

