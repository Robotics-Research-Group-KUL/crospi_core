
#if !defined(CROSPI_BOOST_PP_IS_ITERATING)

///// header body

#ifndef CROSPI_BOOST_MPL_QUOTE_HPP_INCLUDED
#define CROSPI_BOOST_MPL_QUOTE_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2008
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#if !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)
#   include <crospi_boost/mpl/void.hpp>
#   include <crospi_boost/mpl/aux_/has_type.hpp>
#endif

#include <crospi_boost/mpl/aux_/config/bcc.hpp>
#include <crospi_boost/mpl/aux_/config/ttp.hpp>

#if defined(CROSPI_BOOST_MPL_CFG_NO_TEMPLATE_TEMPLATE_PARAMETERS) \
    && !defined(CROSPI_BOOST_MPL_CFG_BCC590_WORKAROUNDS)
#   define CROSPI_BOOST_MPL_CFG_NO_QUOTE_TEMPLATE
#endif

#if !defined(CROSPI_BOOST_MPL_CFG_NO_IMPLICIT_METAFUNCTIONS) \
    && defined(CROSPI_BOOST_MPL_CFG_NO_HAS_XXX)
#   define CROSPI_BOOST_MPL_CFG_NO_IMPLICIT_METAFUNCTIONS
#endif

#include <crospi_boost/mpl/aux_/config/use_preprocessed.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS) \
 && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)

#   define CROSPI_BOOST_MPL_PREPROCESSED_HEADER quote.hpp
#   include <crospi_boost/mpl/aux_/include_preprocessed.hpp>

#else

#   include <crospi_boost/mpl/limits/arity.hpp>
#   include <crospi_boost/mpl/aux_/preprocessor/params.hpp>
#   include <crospi_boost/mpl/aux_/config/ctps.hpp>
#   include <crospi_boost/mpl/aux_/config/workaround.hpp>

#   include <crospi_boost/preprocessor/iterate.hpp>
#   include <crospi_boost/preprocessor/cat.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_QUOTE_TEMPLATE)

namespace crospi_boost { namespace mpl {

#if !defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)

template< typename T, bool has_type_ >
struct quote_impl
// GCC has a problem with metafunction forwarding when T is a
// specialization of a template called 'type'.
# if CROSPI_BOOST_WORKAROUND(__GNUC__, CROSPI_BOOST_TESTED_AT(4)) \
    && CROSPI_BOOST_WORKAROUND(__GNUC_MINOR__, CROSPI_BOOST_TESTED_AT(0)) \
    && CROSPI_BOOST_WORKAROUND(__GNUC_PATCHLEVEL__, CROSPI_BOOST_TESTED_AT(2))
{
    typedef typename T::type type;
};
# else 
    : T
{
};
# endif 

template< typename T >
struct quote_impl<T,false>
{
    typedef T type;
};

#else // CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION

template< bool > struct quote_impl
{
    template< typename T > struct result_
        : T
    {
    };
};

template<> struct quote_impl<false>
{
    template< typename T > struct result_
    {
        typedef T type;
    };
};

#endif 

#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,(1, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, <crospi_boost/mpl/quote.hpp>))
#include CROSPI_BOOST_PP_ITERATE()

}}

#endif // CROSPI_BOOST_MPL_CFG_NO_QUOTE_TEMPLATE

#endif // CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif // CROSPI_BOOST_MPL_QUOTE_HPP_INCLUDED

///// iteration

#else
#define i_ CROSPI_BOOST_PP_FRAME_ITERATION(1)

template<
      template< CROSPI_BOOST_MPL_PP_PARAMS(i_, typename P) > class F
    , typename Tag = void_
    >
struct CROSPI_BOOST_PP_CAT(quote,i_)
{
    template< CROSPI_BOOST_MPL_PP_PARAMS(i_, typename U) > struct apply
#if defined(CROSPI_BOOST_MPL_CFG_BCC590_WORKAROUNDS)
    {
        typedef typename quote_impl<
              F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) >
            , aux::has_type< F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) > >::value
            >::type type;
    };
#elif !defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        : quote_impl<
              F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) >
            , aux::has_type< F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) > >::value
            >
    {
    };
#else
        : quote_impl< aux::has_type< F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) > >::value >
            ::template result_< F< CROSPI_BOOST_MPL_PP_PARAMS(i_, U) > >
    {
    };
#endif
};

#undef i_
#endif // CROSPI_BOOST_PP_IS_ITERATING
