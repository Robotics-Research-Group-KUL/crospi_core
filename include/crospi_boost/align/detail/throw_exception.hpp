/*
Copyright 2019 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_DETAIL_THROW_EXCEPTION_HPP
#define CROSPI_BOOST_ALIGN_DETAIL_THROW_EXCEPTION_HPP

#include <crospi_boost/config.hpp>
#if defined(CROSPI_BOOST_NO_EXCEPTIONS)
#include <exception>
#endif

namespace crospi_boost {

#if defined(CROSPI_BOOST_NO_EXCEPTIONS)
CROSPI_BOOST_NORETURN void throw_exception(const std::exception&);
#endif

namespace alignment {
namespace detail {

#if !defined(CROSPI_BOOST_NO_EXCEPTIONS)
template<class E>
CROSPI_BOOST_NORETURN inline void
throw_exception(const E& error)
{
    throw error;
}
#else
CROSPI_BOOST_NORETURN inline void
throw_exception(const std::exception& error)
{
    crospi_boost::throw_exception(error);
}
#endif

} /* detail */
} /* alignment */
} /* boost */

#endif
