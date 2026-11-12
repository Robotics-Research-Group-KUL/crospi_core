
//  (C) Copyright John Maddock 2010. 
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef CROSPI_BOOST_TT_IS_RVALUE_REFERENCE_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_RVALUE_REFERENCE_HPP_INCLUDED

#include <crospi_boost/config.hpp>
#include <crospi_boost/type_traits/integral_constant.hpp>

namespace crospi_boost {

template <class T> struct is_rvalue_reference : public false_type {};
#ifndef CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES
template <class T> struct is_rvalue_reference<T&&> : public true_type {};
#endif

} // namespace crospi_boost

#if !defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES) && defined(CROSPI_BOOST_MSVC) && CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1700)
#include <crospi_boost/type_traits/detail/is_rvalue_reference_msvc10_fix.hpp>
#endif

#endif // CROSPI_BOOST_TT_IS_REFERENCE_HPP_INCLUDED

