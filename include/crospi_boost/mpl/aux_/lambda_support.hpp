
#ifndef CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/config/lambda.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT)

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) /**/
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i,name,params) /**/

#else

#   include <crospi_boost/mpl/int_fwd.hpp>
#   include <crospi_boost/mpl/aux_/yes_no.hpp>
#   include <crospi_boost/mpl/aux_/na_fwd.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/enum.hpp>
#   include <crospi_boost/mpl/aux_/config/msvc.hpp>
#   include <crospi_boost/mpl/aux_/config/workaround.hpp>

#   include <crospi_boost/preprocessor/tuple/to_list.hpp>
#   include <crospi_boost/preprocessor/list/for_each_i.hpp>
#   include <crospi_boost/preprocessor/inc.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_ARG_TYPEDEF_FUNC(R,typedef_,i,param) \
    typedef_ param CROSPI_BOOST_PP_CAT(arg,CROSPI_BOOST_PP_INC(i)); \
    /**/

// agurt, 07/mar/03: restore an old revision for the sake of SGI MIPSpro C++
#if CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 238) 

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i, name, params) \
    typedef CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::int_<i> arity; \
    CROSPI_BOOST_PP_LIST_FOR_EACH_I_R( \
          1 \
        , CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_ARG_TYPEDEF_FUNC \
        , typedef \
        , CROSPI_BOOST_PP_TUPLE_TO_LIST(i,params) \
        ) \
    struct rebind \
    { \
        template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename U) > struct apply \
            : name< CROSPI_BOOST_MPL_PP_PARAMS(i,U) > \
        { \
        }; \
    }; \
    /**/

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i, name, params) \
    /**/

#elif CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 244) && !defined(CROSPI_BOOST_INTEL_CXX_VERSION)
// agurt, 18/jan/03: old EDG-based compilers actually enforce 11.4 para 9
// (in strict mode), so we have to provide an alternative to the 
// MSVC-optimized implementation

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
    typedef CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::int_<i> arity; \
    CROSPI_BOOST_PP_LIST_FOR_EACH_I_R( \
          1 \
        , CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_ARG_TYPEDEF_FUNC \
        , typedef \
        , CROSPI_BOOST_PP_TUPLE_TO_LIST(i,params) \
        ) \
    struct rebind; \
/**/

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i, name, params) \
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
}; \
template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename T) > \
struct name<CROSPI_BOOST_MPL_PP_PARAMS(i,T)>::rebind \
{ \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename U) > struct apply \
        : name< CROSPI_BOOST_MPL_PP_PARAMS(i,U) > \
    { \
    }; \
/**/

#else // __EDG_VERSION__

namespace crospi_boost { namespace mpl { namespace aux {
template< typename T > struct has_rebind_tag;
}}}

#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
    typedef CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::int_<i> arity; \
    CROSPI_BOOST_PP_LIST_FOR_EACH_I_R( \
          1 \
        , CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_ARG_TYPEDEF_FUNC \
        , typedef \
        , CROSPI_BOOST_PP_TUPLE_TO_LIST(i,params) \
        ) \
    friend class CROSPI_BOOST_PP_CAT(name,_rebind); \
    typedef CROSPI_BOOST_PP_CAT(name,_rebind) rebind; \
/**/

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610))
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HAS_REBIND(i, name, params) \
template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename T) > \
::crospi_boost::mpl::aux::yes_tag operator|( \
      ::crospi_boost::mpl::aux::has_rebind_tag<int> \
    , name<CROSPI_BOOST_MPL_PP_PARAMS(i,T)>* \
    ); \
::crospi_boost::mpl::aux::no_tag operator|( \
      ::crospi_boost::mpl::aux::has_rebind_tag<int> \
    , name< CROSPI_BOOST_MPL_PP_ENUM(i,::crospi_boost::mpl::na) >* \
    ); \
/**/
#elif !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1300)
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HAS_REBIND(i, name, params) \
template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename T) > \
::crospi_boost::mpl::aux::yes_tag operator|( \
      ::crospi_boost::mpl::aux::has_rebind_tag<int> \
    , ::crospi_boost::mpl::aux::has_rebind_tag< name<CROSPI_BOOST_MPL_PP_PARAMS(i,T)> >* \
    ); \
/**/
#else
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HAS_REBIND(i, name, params) /**/
#endif

#   if !defined(CROSPI_BOOST_BORLANDC)
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i, name, params) \
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
}; \
CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HAS_REBIND(i, name, params) \
class CROSPI_BOOST_PP_CAT(name,_rebind) \
{ \
 public: \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename U) > struct apply \
        : name< CROSPI_BOOST_MPL_PP_PARAMS(i,U) > \
    { \
    }; \
/**/
#   else
#   define CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT(i, name, params) \
    CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_SPEC(i, name, params) \
}; \
CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HAS_REBIND(i, name, params) \
class CROSPI_BOOST_PP_CAT(name,_rebind) \
{ \
 public: \
    template< CROSPI_BOOST_MPL_PP_PARAMS(i,typename U) > struct apply \
    { \
        typedef typename name< CROSPI_BOOST_MPL_PP_PARAMS(i,U) >::type type; \
    }; \
/**/
#   endif // CROSPI_BOOST_BORLANDC

#endif // __EDG_VERSION__

#endif // CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT

#endif // CROSPI_BOOST_MPL_AUX_LAMBDA_SUPPORT_HPP_INCLUDED
