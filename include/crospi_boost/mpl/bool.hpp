
#ifndef CROSPI_BOOST_MPL_BOOL_HPP_INCLUDED
#define CROSPI_BOOST_MPL_BOOL_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/bool_fwd.hpp>
#include <crospi_boost/mpl/integral_c_tag.hpp>
#include <crospi_boost/mpl/aux_/config/static_constant.hpp>

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN

template< bool C_ > struct bool_
{
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = C_);
    typedef integral_c_tag tag;
    typedef bool_ type;
    typedef bool value_type;
    CROSPI_BOOST_CONSTEXPR operator bool() const { return this->value; }
};

#if !defined(CROSPI_BOOST_NO_INCLASS_MEMBER_INITIALIZATION)
template< bool C_ >
bool const bool_<C_>::value;
#endif

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE

#endif // CROSPI_BOOST_MPL_BOOL_HPP_INCLUDED
