
#ifndef CROSPI_BOOST_MPL_AUX_IS_MSVC_ETI_ARG_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_IS_MSVC_ETI_ARG_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/yes_no.hpp>
#include <crospi_boost/mpl/aux_/config/eti.hpp>
#include <crospi_boost/mpl/aux_/config/static_constant.hpp>

namespace crospi_boost { namespace mpl { namespace aux {

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG)

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_60_ETI_BUG)

template< typename T >
struct is_msvc_eti_arg
{ 
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = false);
};

#else // CROSPI_BOOST_MPL_CFG_MSVC_60_ETI_BUG

struct eti_int_convertible
{
    eti_int_convertible(int);
};

template< typename T >
struct is_msvc_eti_arg
{ 
    static no_tag test(...);
    static yes_tag test(eti_int_convertible);
    static T& get();

    CROSPI_BOOST_STATIC_CONSTANT(bool, value = 
          sizeof(test(get())) == sizeof(yes_tag)
        );
};

#endif

template<>
struct is_msvc_eti_arg<int>
{ 
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = true);
};

#endif // CROSPI_BOOST_MPL_CFG_MSVC_ETI_BUG

}}}

#endif // CROSPI_BOOST_MPL_AUX_IS_MSVC_ETI_ARG_HPP_INCLUDED
