
#ifndef CROSPI_BOOST_MPL_NOT_HPP_INCLUDED
#define CROSPI_BOOST_MPL_NOT_HPP_INCLUDED

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

#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/aux_/nttp_decl.hpp>
#include <crospi_boost/mpl/aux_/nested_type_wknd.hpp>
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/lambda_support.hpp>

namespace crospi_boost { namespace mpl {

namespace aux {

template< CROSPI_BOOST_MPL_AUX_NTTP_DECL(long, C_) > // 'long' is intentional here
struct not_impl
    : bool_<!C_>
{
};

} // namespace aux


template<
      typename CROSPI_BOOST_MPL_AUX_NA_PARAM(T)
    >
struct not_
    : aux::not_impl<
          CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND(T)::value
        >
{
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(1,not_,(T))
};

CROSPI_BOOST_MPL_AUX_NA_SPEC(1,not_)

}}

#endif // CROSPI_BOOST_MPL_NOT_HPP_INCLUDED
