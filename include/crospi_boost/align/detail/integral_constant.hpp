/*
Copyright 2014-2016 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_DETAIL_INTEGRAL_CONSTANT_HPP
#define CROSPI_BOOST_ALIGN_DETAIL_INTEGRAL_CONSTANT_HPP

#include <crospi_boost/config.hpp>

#if !defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS)
#include <type_traits>
#endif

namespace crospi_boost {
namespace alignment {
namespace detail {

#if !defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS)
using std::integral_constant;
using std::true_type;
using std::false_type;
#else
template<class T, T Value>
struct integral_constant {
    typedef T value_type;
    typedef integral_constant type;

    CROSPI_BOOST_CONSTEXPR operator value_type() const CROSPI_BOOST_NOEXCEPT {
        return Value;
    }

    CROSPI_BOOST_CONSTEXPR value_type operator()() const CROSPI_BOOST_NOEXCEPT {
        return Value;
    }

    CROSPI_BOOST_STATIC_CONSTEXPR T value = Value;
};

template<class T, T Value>
CROSPI_BOOST_CONSTEXPR_OR_CONST T integral_constant<T, Value>::value;

typedef integral_constant<bool, true> true_type;
typedef integral_constant<bool, false> false_type;
#endif

} /* detail */
} /* alignment */
} /* boost */

#endif
