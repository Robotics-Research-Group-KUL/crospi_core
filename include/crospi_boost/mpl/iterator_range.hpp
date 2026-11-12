
#ifndef CROSPI_BOOST_MPL_ITERATOR_RANGE_HPP_INCLUDED
#define CROSPI_BOOST_MPL_ITERATOR_RANGE_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>

namespace crospi_boost { namespace mpl {

struct iterator_range_tag;

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(First)
    , typename CROSPI_BOOST_MPL_AUX_NA_PARAM(Last)
    >
struct iterator_range
{
    typedef iterator_range_tag tag;
    typedef iterator_range type;
    typedef First begin;
    typedef Last end;

    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(2,iterator_range,(First,Last))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC(2, iterator_range)

}}

#endif // CROSPI_BOOST_MPL_ITERATOR_RANGE_HPP_INCLUDED
