
//  (C) Copyright Dave Abrahams, Steve Cleary, Beman Dawes, Howard
//  Hinnant & John Maddock 2000.  
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.


#ifndef CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_HPP_INCLUDED

#include <crospi_boost/type_traits/detail/config.hpp>

#ifdef CROSPI_BOOST_TT_HAS_ACCURATE_IS_FUNCTION

#include <crospi_boost/type_traits/detail/is_member_function_pointer_cxx_11.hpp>

#else

#include <crospi_boost/type_traits/detail/is_member_function_pointer_cxx_03.hpp>

#endif

#endif // CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_HPP_INCLUDED
