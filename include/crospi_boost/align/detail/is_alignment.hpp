/*
Copyright 2014 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_DETAIL_IS_ALIGNMENT_HPP
#define CROSPI_BOOST_ALIGN_DETAIL_IS_ALIGNMENT_HPP

#include <crospi_boost/config.hpp>
#include <cstddef>

namespace crospi_boost {
namespace alignment {
namespace detail {

CROSPI_BOOST_CONSTEXPR inline bool
is_alignment(std::size_t value) CROSPI_BOOST_NOEXCEPT
{
    return (value > 0) && ((value & (value - 1)) == 0);
}

} /* detail */
} /* alignment */
} /* boost */

#endif
