
#ifndef CROSPI_BOOST_MPL_PAIR_HPP_INCLUDED
#define CROSPI_BOOST_MPL_PAIR_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/msvc_eti_base.hpp>
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>
#include <crospi_boost/mpl/aux_/config/eti.hpp>

namespace crospi_boost { namespace mpl {

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T1)
    , typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T2)
    >
struct pair
{
    typedef pair type;
    typedef T1 first;
    typedef T2 second;

    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(2,pair,(T1,T2))
};

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(P)
    >
struct first
{
#if !defined(CROSPI_BOOST_MPL_CFG_MSVC_70_ETI_BUG)
    typedef typename P::first type;
#else
    typedef typename aux::msvc_eti_base<P>::first type;
#endif
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(1,first,(P))
};

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(P)
    >
struct second
{
#if !defined(CROSPI_BOOST_MPL_CFG_MSVC_70_ETI_BUG)
    typedef typename P::second type;
#else
    typedef typename aux::msvc_eti_base<P>::second type;
#endif
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(1,second,(P))
};


CROSPI_BOOST_MPL_AUX_NA_SPEC_NO_ETI(2, pair)
CROSPI_BOOST_MPL_AUX_NA_SPEC(1, first)
CROSPI_BOOST_MPL_AUX_NA_SPEC(1, second)

}}

#endif // CROSPI_BOOST_MPL_PAIR_HPP_INCLUDED
