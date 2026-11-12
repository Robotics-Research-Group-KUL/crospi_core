
//  (C) Copyright Dave Abrahams, Steve Cleary, Beman Dawes, Howard
//  Hinnant & John Maddock 2000.  
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.


#ifndef CROSPI_BOOST_TT_REMOVE_CONST_HPP_INCLUDED
#define CROSPI_BOOST_TT_REMOVE_CONST_HPP_INCLUDED

#include <crospi_boost/config.hpp>
#include <cstddef> // size_t
#include <crospi_boost/detail/workaround.hpp>

namespace crospi_boost {

   //  convert a type T to a non-cv-qualified type - remove_const<T>
   template <class T> struct remove_const{ typedef T type; };
   template <class T> struct remove_const<T const>{ typedef T type; };

#if !defined(CROSPI_BOOST_NO_ARRAY_TYPE_SPECIALIZATIONS)
   template <class T, std::size_t N> struct remove_const<T const[N]>{ typedef T type[N]; };
#if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, < 0x600) && !defined(__IBMCPP__) &&  !CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
   template <class T> struct remove_const<T const[]>{ typedef T type[]; };
#endif
#endif

#if !defined(CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES)

   template <class T> using remove_const_t = typename remove_const<T>::type;

#endif

} // namespace crospi_boost

#endif // CROSPI_BOOST_TT_REMOVE_CONST_HPP_INCLUDED
