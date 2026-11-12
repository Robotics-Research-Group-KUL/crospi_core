
#ifndef CROSPI_BOOST_MPL_ALWAYS_HPP_INCLUDED
#define CROSPI_BOOST_MPL_ALWAYS_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/preprocessor/default_params.hpp>
#include <crospi_boost/mpl/aux_/na.hpp>
#include <crospi_boost/mpl/aux_/arity_spec.hpp>

namespace crospi_boost { namespace mpl {

template< typename Value > struct always
{
    template<
        CROSPI_BOOST_MPL_PP_DEFAULT_PARAMS(CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, typename T, na)
        >
    struct apply
    {
        typedef Value type;
    };
};

CROSPI_BOOST_MPL_AUX_ARITY_SPEC(0, always)

}}

#endif // CROSPI_BOOST_MPL_ALWAYS_HPP_INCLUDED
