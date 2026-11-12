
#ifndef CROSPI_BOOST_MPL_AUX_PREPROCESSOR_ENUM_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_PREPROCESSOR_ENUM_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/config/preprocessor.hpp>

// CROSPI_BOOST_MPL_PP_ENUM(0,int): <nothing>
// CROSPI_BOOST_MPL_PP_ENUM(1,int): int
// CROSPI_BOOST_MPL_PP_ENUM(2,int): int, int
// CROSPI_BOOST_MPL_PP_ENUM(n,int): int, int, .., int

#if !defined(CROSPI_BOOST_MPL_CFG_NO_OWN_PP_PRIMITIVES)

#   include <crospi_boost/preprocessor/cat.hpp>

#   define CROSPI_BOOST_MPL_PP_ENUM(n, param) \
    CROSPI_BOOST_PP_CAT(CROSPI_BOOST_MPL_PP_ENUM_,n)(param) \
    /**/
    
#   define CROSPI_BOOST_MPL_PP_ENUM_Z(z_ignored, n, param) \
    CROSPI_BOOST_PP_CAT(CROSPI_BOOST_MPL_PP_ENUM_,n)(param) \
    /**/
    
#   define CROSPI_BOOST_MPL_PP_ENUM_0(p)
#   define CROSPI_BOOST_MPL_PP_ENUM_1(p) p
#   define CROSPI_BOOST_MPL_PP_ENUM_2(p) p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_3(p) p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_4(p) p,p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_5(p) p,p,p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_6(p) p,p,p,p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_7(p) p,p,p,p,p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_8(p) p,p,p,p,p,p,p,p
#   define CROSPI_BOOST_MPL_PP_ENUM_9(p) p,p,p,p,p,p,p,p,p

#else

#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/repeat.hpp>

#   define CROSPI_BOOST_MPL_PP_AUX_ENUM_FUNC(unused, i, param) \
    CROSPI_BOOST_PP_COMMA_IF(i) param \
    /**/

#   define CROSPI_BOOST_MPL_PP_ENUM(n, param) \
    CROSPI_BOOST_PP_REPEAT( \
          n \
        , CROSPI_BOOST_MPL_PP_AUX_ENUM_FUNC \
        , param \
        ) \
    /**/

#   define CROSPI_BOOST_MPL_PP_ENUM_Z(z, n, param) \
    CROSPI_BOOST_PP_REPEAT_ ## z( \
          n \
        , CROSPI_BOOST_MPL_PP_AUX_ENUM_FUNC \
        , param \
        ) \
    /**/

#endif

#endif // CROSPI_BOOST_MPL_AUX_PREPROCESSOR_ENUM_HPP_INCLUDED
