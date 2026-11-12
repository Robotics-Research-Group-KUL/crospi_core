//  Copyright (C) 2023 Tim Blechmann
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_LOCKFREE_DETAIL_USES_OPTIONAL_HPP
#define CROSPI_BOOST_LOCKFREE_DETAIL_USES_OPTIONAL_HPP

#include <crospi_boost/config.hpp>

#ifndef CROSPI_BOOST_NO_CXX17_HDR_OPTIONAL

#    include <optional>

namespace crospi_boost { namespace lockfree {

struct uses_optional_t
{};

#    ifdef CROSPI_BOOST_NO_CXX17_INLINE_VARIABLES
static
#    else
inline
#    endif
    constexpr uses_optional_t uses_optional;

}} // namespace crospi_boost::lockfree

#endif

#endif /* CROSPI_BOOST_LOCKFREE_DETAIL_USES_OPTIONAL_HPP */
