
//  (C) Copyright Steve Cleary, Beman Dawes, Howard Hinnant & John Maddock 2000.
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef CROSPI_BOOST_TT_IS_SCALAR_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_SCALAR_HPP_INCLUDED

#include <crospi_boost/type_traits/is_arithmetic.hpp>
#include <crospi_boost/type_traits/is_enum.hpp>
#include <crospi_boost/type_traits/is_pointer.hpp>
#include <crospi_boost/type_traits/is_member_pointer.hpp>
#include <crospi_boost/config.hpp>

namespace crospi_boost {

template <typename T>
struct is_scalar
   : public integral_constant<bool, ::crospi_boost::is_arithmetic<T>::value || ::crospi_boost::is_enum<T>::value || ::crospi_boost::is_pointer<T>::value || ::crospi_boost::is_member_pointer<T>::value>
{};

} // namespace crospi_boost

#endif // CROSPI_BOOST_TT_IS_SCALAR_HPP_INCLUDED
