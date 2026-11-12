
#ifndef CROSPI_BOOST_MPL_FIND_IF_HPP_INCLUDED
#define CROSPI_BOOST_MPL_FIND_IF_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/find_if_pred.hpp>
#include <crospi_boost/mpl/arg.hpp>
#include <crospi_boost/mpl/iter_fold_if.hpp>
#include <crospi_boost/mpl/aux_/common_name_wknd.hpp>
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>

namespace crospi_boost { namespace mpl {

CROSPI_BOOST_MPL_AUX_COMMON_NAME_WKND(find_if)

template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(Sequence)
    , typename CROSPI_BOOST_MPL_AUX_NA_PARAM(Predicate)
    >
struct find_if
{
    typedef typename iter_fold_if<
          Sequence
        , void
        , mpl::arg<1> // ignore
        , protect< aux::find_if_pred<Predicate> >
        >::type result_;

    typedef typename second<result_>::type type;

    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(2,find_if,(Sequence,Predicate))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC(2,find_if)

}}

#endif // CROSPI_BOOST_MPL_FIND_IF_HPP_INCLUDED
