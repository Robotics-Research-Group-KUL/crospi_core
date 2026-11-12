
#ifndef CROSPI_BOOST_MPL_IDENTITY_HPP_INCLUDED
#define CROSPI_BOOST_MPL_IDENTITY_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>

namespace crospi_boost { namespace mpl {

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T)
    >
struct identity
{
    typedef T type;
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(1, identity, (T))
};

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T)
    >
struct make_identity
{
    typedef identity<T> type;
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(1, make_identity, (T))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC_NO_ETI(1, identity)
CROSPI_BOOST_MPL_AUX_NA_SPEC_NO_ETI(1, make_identity)

}}

#endif // CROSPI_BOOST_MPL_IDENTITY_HPP_INCLUDED
