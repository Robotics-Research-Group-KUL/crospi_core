
#ifndef CROSPI_BOOST_MPL_DISTANCE_HPP_INCLUDED
#define CROSPI_BOOST_MPL_DISTANCE_HPP_INCLUDED

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

#include <crospi_boost/mpl/distance_fwd.hpp>
#include <crospi_boost/mpl/iter_fold.hpp>
#include <crospi_boost/mpl/iterator_range.hpp>
#include <crospi_boost/mpl/long.hpp>
#include <crospi_boost/mpl/next.hpp>
#include <crospi_boost/mpl/tag.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/aux_/msvc_eti_base.hpp>
#include <crospi_boost/mpl/aux_/value_wknd.hpp>
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/config/forwarding.hpp>
#include <crospi_boost/mpl/aux_/config/static_constant.hpp>


namespace crospi_boost { namespace mpl {

// default implementation for forward/bidirectional iterators
template< typename Tag > struct distance_impl
{
    template< typename First, typename Last > struct apply
#if !defined(CROSPI_BOOST_MPL_CFG_NO_NESTED_FORWARDING)
        : aux::msvc_eti_base< typename iter_fold<
              iterator_range<First,Last>
            , mpl::long_<0>
            , next<>
            >::type >
    {
#else
    {
        typedef typename iter_fold<
              iterator_range<First,Last>
            , mpl::long_<0>
            , next<>
            >::type type;
        
        CROSPI_BOOST_STATIC_CONSTANT(long, value =
              (iter_fold<
                  iterator_range<First,Last>
                , mpl::long_<0>
                , next<>
                >::type::value)
            );
#endif
    };
};

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(First)
    , typename CROSPI_BOOST_MPL_AUX_NA_PARAM(Last)
    >
struct distance
    : distance_impl< typename tag<First>::type >
        ::template apply<First, Last>
{
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(2, distance, (First, Last))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC(2, distance)

}}

#endif // CROSPI_BOOST_MPL_DISTANCE_HPP_INCLUDED
