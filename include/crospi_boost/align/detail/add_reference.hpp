/*
Copyright 2019 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_DETAIL_ADD_REFERENCE_HPP
#define CROSPI_BOOST_ALIGN_DETAIL_ADD_REFERENCE_HPP

#include <crospi_boost/config.hpp>

#if !defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS)
#include <type_traits>
#endif

namespace crospi_boost {
namespace alignment {
namespace detail {

#if !defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS)
using std::add_lvalue_reference;
#else
template<class T>
struct add_lvalue_reference {
    typedef T& type;
};

template<>
struct add_lvalue_reference<void> {
    typedef void type;
};

template<>
struct add_lvalue_reference<const void> {
    typedef const void type;
};
#endif

} /* detail */
} /* alignment */
} /* boost */

#endif
