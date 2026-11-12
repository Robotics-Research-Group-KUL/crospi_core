
#ifndef CROSPI_BOOST_MPL_AUX_MSVC_ETI_BASE_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_MSVC_ETI_BASE_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/is_msvc_eti_arg.hpp>
#include <crospi_boost/mpl/aux_/config/eti.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

namespace crospi_boost { namespace mpl { namespace aux {

#if defined(CROSPI_BOOST_MPL_CFG_MSVC_70_ETI_BUG)

template< bool > struct msvc_eti_base_impl
{
    template< typename T > struct result_
        : T
    {
        typedef T type;
    };
};

template<> struct msvc_eti_base_impl<true>
{
    template< typename T > struct result_
    {
        typedef result_ type;
        typedef result_ first;
        typedef result_ second;
        typedef result_ tag;
        enum { value = 0 };
    };
};

template< typename T > struct msvc_eti_base
    : msvc_eti_base_impl< is_msvc_eti_arg<T>::value >
        ::template result_<T>
{
};

#else // !CROSPI_BOOST_MPL_CFG_MSVC_70_ETI_BUG

template< typename T > struct msvc_eti_base
    : T
{
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MPL_CFG_GCC, CROSPI_BOOST_TESTED_AT(0x0304))
    msvc_eti_base();
#endif
    typedef T type;
};

#endif 

template<> struct msvc_eti_base<int>
{
    typedef msvc_eti_base type;
    typedef msvc_eti_base first;
    typedef msvc_eti_base second;
    typedef msvc_eti_base tag;
    enum { value = 0 };
};

}}}

#endif // CROSPI_BOOST_MPL_AUX_MSVC_ETI_BASE_HPP_INCLUDED
