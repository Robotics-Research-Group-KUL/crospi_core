
#ifndef CROSPI_BOOST_MPL_AUX_PREPROCESSOR_PARAMS_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_PREPROCESSOR_PARAMS_HPP_INCLUDED

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

// CROSPI_BOOST_MPL_PP_PARAMS(0,T): <nothing>
// CROSPI_BOOST_MPL_PP_PARAMS(1,T): T1
// CROSPI_BOOST_MPL_PP_PARAMS(2,T): T1, T2
// CROSPI_BOOST_MPL_PP_PARAMS(n,T): T1, T2, .., Tn

#if !defined(CROSPI_BOOST_MPL_CFG_NO_OWN_PP_PRIMITIVES)

#   include <crospi_boost/preprocessor/cat.hpp>

#   define CROSPI_BOOST_MPL_PP_PARAMS(n,p) \
    CROSPI_BOOST_PP_CAT(CROSPI_BOOST_MPL_PP_PARAMS_,n)(p) \
    /**/

#   define CROSPI_BOOST_MPL_PP_PARAMS_Z(z_ignored,n,p) \
    CROSPI_BOOST_PP_CAT(CROSPI_BOOST_MPL_PP_PARAMS_,n)(p) \
    /**/

#   define CROSPI_BOOST_MPL_PP_PARAMS_0(p)
#   define CROSPI_BOOST_MPL_PP_PARAMS_1(p) p##1
#   define CROSPI_BOOST_MPL_PP_PARAMS_2(p) p##1,p##2
#   define CROSPI_BOOST_MPL_PP_PARAMS_3(p) p##1,p##2,p##3
#   define CROSPI_BOOST_MPL_PP_PARAMS_4(p) p##1,p##2,p##3,p##4
#   define CROSPI_BOOST_MPL_PP_PARAMS_5(p) p##1,p##2,p##3,p##4,p##5
#   define CROSPI_BOOST_MPL_PP_PARAMS_6(p) p##1,p##2,p##3,p##4,p##5,p##6
#   define CROSPI_BOOST_MPL_PP_PARAMS_7(p) p##1,p##2,p##3,p##4,p##5,p##6,p##7
#   define CROSPI_BOOST_MPL_PP_PARAMS_8(p) p##1,p##2,p##3,p##4,p##5,p##6,p##7,p##8
#   define CROSPI_BOOST_MPL_PP_PARAMS_9(p) p##1,p##2,p##3,p##4,p##5,p##6,p##7,p##8,p##9

#else

#   include <crospi_boost/preprocessor/comma_if.hpp>
#   include <crospi_boost/preprocessor/repeat.hpp>
#   include <crospi_boost/preprocessor/inc.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

#   define CROSPI_BOOST_MPL_PP_AUX_PARAM_FUNC(unused, i, param) \
    CROSPI_BOOST_PP_COMMA_IF(i) \
    CROSPI_BOOST_PP_CAT(param, CROSPI_BOOST_PP_INC(i)) \
    /**/

#   define CROSPI_BOOST_MPL_PP_PARAMS(n, param) \
    CROSPI_BOOST_PP_REPEAT( \
          n \
        , CROSPI_BOOST_MPL_PP_AUX_PARAM_FUNC \
        , param \
        ) \
    /**/

#   define CROSPI_BOOST_MPL_PP_PARAMS_Z(z, n, param) \
    CROSPI_BOOST_PP_REPEAT_ ## z( \
          n \
        , CROSPI_BOOST_MPL_PP_AUX_PARAM_FUNC \
        , param \
        ) \
    /**/

#endif 

#endif // CROSPI_BOOST_MPL_AUX_PREPROCESSOR_PARAMS_HPP_INCLUDED
