
#ifndef CROSPI_BOOST_MPL_FIND_HPP_INCLUDED
#define CROSPI_BOOST_MPL_FIND_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2002
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/find_if.hpp>
#include <crospi_boost/mpl/same_as.hpp>
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>

namespace crospi_boost { namespace mpl {

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(Sequence)
    , typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T)
    >
struct find
    : find_if< Sequence,same_as<T> >
{
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(2,find,(Sequence,T))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC(2, find)

}}

#endif // CROSPI_BOOST_MPL_FIND_HPP_INCLUDED
