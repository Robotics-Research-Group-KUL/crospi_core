
#ifndef CROSPI_BOOST_MPL_HAS_XXX_HPP_INCLUDED
#define CROSPI_BOOST_MPL_HAS_XXX_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2002-2006
// Copyright David Abrahams 2002-2003
// Copyright Daniel Walker 2007
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
#include <crospi_boost/mpl/aux_/na_spec.hpp>
#include <crospi_boost/mpl/aux_/type_wrapper.hpp>
#include <crospi_boost/mpl/aux_/yes_no.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/has_xxx.hpp>
#include <crospi_boost/mpl/aux_/config/msvc_typename.hpp>
#include <crospi_boost/mpl/aux_/config/msvc.hpp>
#include <crospi_boost/mpl/aux_/config/static_constant.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#include <crospi_boost/preprocessor/array/elem.hpp>
#include <crospi_boost/preprocessor/cat.hpp>
#include <crospi_boost/preprocessor/control/if.hpp>
#include <crospi_boost/preprocessor/repetition/enum_params.hpp>
#include <crospi_boost/preprocessor/repetition/enum_trailing_params.hpp>

#if CROSPI_BOOST_WORKAROUND( CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x590) )
# include <crospi_boost/type_traits/is_class.hpp>
#endif

#if !defined(CROSPI_BOOST_MPL_CFG_NO_HAS_XXX)

#   if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1300)

// agurt, 11/sep/02: MSVC-specific version (< 7.1), based on a USENET 
// newsgroup's posting by John Madsen (comp.lang.c++.moderated, 
// 1999-11-12 19:17:06 GMT); the code is _not_ standard-conforming, but 
// it works way more reliably than the SFINAE-based implementation

// Modified dwa 8/Oct/02 to handle reference types.

#   include <crospi_boost/mpl/if.hpp>
#   include <crospi_boost/mpl/bool.hpp>

namespace crospi_boost { namespace mpl { namespace aux {

struct has_xxx_tag;

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1300)
template< typename U > struct msvc_incomplete_array
{
    typedef char (&type)[sizeof(U) + 1];
};
#endif

template< typename T >
struct msvc_is_incomplete
{
    // MSVC is capable of some kinds of SFINAE.  If U is an incomplete
    // type, it won't pick the second overload
    static char tester(...);

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1300)
    template< typename U >
    static typename msvc_incomplete_array<U>::type tester(type_wrapper<U>);
#else
    template< typename U >
    static char (& tester(type_wrapper<U>) )[sizeof(U)+1];
#endif 
    
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = 
          sizeof(tester(type_wrapper<T>())) == 1
        );
};

template<>
struct msvc_is_incomplete<int>
{
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = false);
};

}}}

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF_(trait, name, default_) \
template< typename T, typename name = ::crospi_boost::mpl::aux::has_xxx_tag > \
struct CROSPI_BOOST_PP_CAT(trait,_impl) : T \
{ \
    static crospi_boost::mpl::aux::no_tag \
    test(void(*)(::crospi_boost::mpl::aux::has_xxx_tag)); \
    \
    static crospi_boost::mpl::aux::yes_tag test(...); \
    \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = \
          sizeof(test(static_cast<void(*)(name)>(0))) \
            != sizeof(crospi_boost::mpl::aux::no_tag) \
        ); \
    typedef crospi_boost::mpl::bool_<value> type; \
}; \
\
template< typename T, typename fallback_ = crospi_boost::mpl::bool_<default_> > \
struct trait \
    : crospi_boost::mpl::if_c< \
          crospi_boost::mpl::aux::msvc_is_incomplete<T>::value \
        , crospi_boost::mpl::bool_<false> \
        , CROSPI_BOOST_PP_CAT(trait,_impl)<T> \
        >::type \
{ \
}; \
\
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, void) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, bool) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, char) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, signed char) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, unsigned char) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, signed short) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, unsigned short) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, signed int) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, unsigned int) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, signed long) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, unsigned long) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, float) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, double) \
CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, long double) \
/**/

#   define CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, T) \
template<> struct trait<T> \
{ \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = false); \
    typedef crospi_boost::mpl::bool_<false> type; \
}; \
/**/

#if !defined(CROSPI_BOOST_NO_INTRINSIC_WCHAR_T)
#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, unused) \
    CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF_(trait, name, unused) \
    CROSPI_BOOST_MPL_AUX_HAS_XXX_TRAIT_SPEC(trait, wchar_t) \
/**/
#else
#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, unused) \
    CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF_(trait, name, unused) \
/**/
#endif


// SFINAE-based implementations below are derived from a USENET newsgroup's 
// posting by Rani Sharoni (comp.lang.c++.moderated, 2002-03-17 07:45:09 PST)

#   elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1400) \
      || (CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, CROSPI_BOOST_TESTED_AT(1800)) && defined(__CUDACC__)) \
      || CROSPI_BOOST_WORKAROUND(__IBMCPP__, <= 700)

// MSVC 7.1 & MSVC 8.0 & VACPP

// agurt, 15/jun/05: replace overload-based SFINAE implementation with SFINAE
// applied to partial specialization to fix some apparently random failures 
// (thanks to Daniel Wallin for researching this!)

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, default_) \
template< typename T > \
struct CROSPI_BOOST_PP_CAT(trait, _msvc_sfinae_helper) \
{ \
    typedef void type; \
};\
\
template< typename T, typename U = void > \
struct CROSPI_BOOST_PP_CAT(trait,_impl_) \
{ \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = false); \
    typedef crospi_boost::mpl::bool_<value> type; \
}; \
\
template< typename T > \
struct CROSPI_BOOST_PP_CAT(trait,_impl_)< \
      T \
    , typename CROSPI_BOOST_PP_CAT(trait, _msvc_sfinae_helper)< typename T::name >::type \
    > \
{ \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = true); \
    typedef crospi_boost::mpl::bool_<value> type; \
}; \
\
template< typename T, typename fallback_ = crospi_boost::mpl::bool_<default_> > \
struct trait \
    : CROSPI_BOOST_PP_CAT(trait,_impl_)<T> \
{ \
}; \
/**/

#   elif CROSPI_BOOST_WORKAROUND( CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x590) )

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_BCB_DEF(trait, trait_tester, name, default_) \
template< typename T, bool IS_CLASS > \
struct trait_tester \
{ \
    CROSPI_BOOST_STATIC_CONSTANT( bool,  value = false ); \
}; \
template< typename T > \
struct trait_tester< T, true > \
{ \
    struct trait_tester_impl \
    { \
        template < class U > \
        static int  resolve( crospi_boost::mpl::aux::type_wrapper<U> const volatile * \
                           , crospi_boost::mpl::aux::type_wrapper<typename U::name >* = 0 ); \
        static char resolve( ... ); \
    }; \
    typedef crospi_boost::mpl::aux::type_wrapper<T> t_; \
    CROSPI_BOOST_STATIC_CONSTANT( bool, value = ( sizeof( trait_tester_impl::resolve( static_cast< t_ * >(0) ) ) == sizeof(int) ) ); \
}; \
template< typename T, typename fallback_ = crospi_boost::mpl::bool_<default_> > \
struct trait           \
{                      \
    CROSPI_BOOST_STATIC_CONSTANT( bool, value = (trait_tester< T, crospi_boost::is_class< T >::value >::value) );     \
    typedef crospi_boost::mpl::bool_< trait< T, fallback_ >::value > type; \
};

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, default_) \
    CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_BCB_DEF( trait \
                                         , CROSPI_BOOST_PP_CAT(trait,_tester)      \
                                         , name       \
                                         , default_ ) \
/**/

#   else // other SFINAE-capable compilers

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, default_) \
template< typename T, typename fallback_ = crospi_boost::mpl::bool_<default_> > \
struct trait \
{ \
    struct gcc_3_2_wknd \
    { \
        template< typename U > \
        static crospi_boost::mpl::aux::yes_tag test( \
              crospi_boost::mpl::aux::type_wrapper<U> const volatile* \
            , crospi_boost::mpl::aux::type_wrapper<CROSPI_BOOST_MSVC_TYPENAME U::name>* = 0 \
            ); \
    \
        static crospi_boost::mpl::aux::no_tag test(...); \
    }; \
    \
    typedef crospi_boost::mpl::aux::type_wrapper<T> t_; \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = \
          sizeof(gcc_3_2_wknd::test(static_cast<t_*>(0))) \
            == sizeof(crospi_boost::mpl::aux::yes_tag) \
        ); \
    typedef crospi_boost::mpl::bool_<value> type; \
}; \
/**/

#   endif // CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1300)


#else // CROSPI_BOOST_MPL_CFG_NO_HAS_XXX

// placeholder implementation

#   define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(trait, name, default_) \
template< typename T, typename fallback_ = crospi_boost::mpl::bool_<default_> > \
struct trait \
{ \
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = fallback_::value); \
    typedef fallback_ type; \
}; \
/**/

#endif

#define CROSPI_BOOST_MPL_HAS_XXX_TRAIT_DEF(name) \
    CROSPI_BOOST_MPL_HAS_XXX_TRAIT_NAMED_DEF(CROSPI_BOOST_PP_CAT(has_,name), name, false) \
/**/


#if !defined(CROSPI_BOOST_MPL_CFG_NO_HAS_XXX_TEMPLATE)

// Create a boolean Metafunction to detect a nested template
// member. This implementation is based on a USENET newsgroup's
// posting by Aleksey Gurtovoy (comp.lang.c++.moderated, 2002-03-19),
// Rani Sharoni's USENET posting cited above, the non-template has_xxx
// implementations above, and discussion on the Boost mailing list.

#   if !defined(CROSPI_BOOST_MPL_HAS_XXX_NO_WRAPPED_TYPES)
#     if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1400)
#       define CROSPI_BOOST_MPL_HAS_XXX_NO_WRAPPED_TYPES 1
#     else
#       define CROSPI_BOOST_MPL_HAS_XXX_NO_WRAPPED_TYPES 0
#     endif
#   endif

#   if !defined(CROSPI_BOOST_MPL_HAS_XXX_NO_EXPLICIT_TEST_FUNCTION)
#     if (defined(CROSPI_BOOST_NO_EXPLICIT_FUNCTION_TEMPLATE_ARGUMENTS))
#       define CROSPI_BOOST_MPL_HAS_XXX_NO_EXPLICIT_TEST_FUNCTION 1
#     else
#       define CROSPI_BOOST_MPL_HAS_XXX_NO_EXPLICIT_TEST_FUNCTION 0
#     endif
#   endif

#   if !defined(CROSPI_BOOST_MPL_HAS_XXX_NEEDS_TEMPLATE_SFINAE)
#     if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1400)
#       define CROSPI_BOOST_MPL_HAS_XXX_NEEDS_TEMPLATE_SFINAE 1
#     else
#       define CROSPI_BOOST_MPL_HAS_XXX_NEEDS_TEMPLATE_SFINAE 0
#     endif
#   endif

// NOTE: Many internal implementation macros take a Boost.Preprocessor
// array argument called args which is of the following form.
//           ( 4, ( trait, name, max_arity, default_ ) )

#   define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_NAME(args) \
      CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_ARRAY_ELEM(0, args) , _introspect) \
    /**/

#   define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME(args, n) \
      CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_ARRAY_ELEM(0, args) , _substitute), n) \
    /**/

#   define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args) \
      CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_ARRAY_ELEM(0, args) , _test) \
    /**/

// Thanks to Guillaume Melquiond for pointing out the need for the
// "substitute" template as an argument to the overloaded test
// functions to get SFINAE to work for member templates with the
// correct name but different number of arguments.
#   define CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_SUBSTITUTE(z, n, args) \
      template< \
          template< CROSPI_BOOST_PP_ENUM_PARAMS_Z(z, CROSPI_BOOST_PP_INC(n), typename V) > class V \
       > \
      struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME(args, n) { \
      }; \
    /**/

#   define CROSPI_BOOST_MPL_HAS_MEMBER_SUBSTITUTE(args, substitute_macro) \
      CROSPI_BOOST_PP_REPEAT( \
          CROSPI_BOOST_PP_ARRAY_ELEM(2, args) \
        , CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_SUBSTITUTE \
        , args \
      ) \
    /**/

#   if !CROSPI_BOOST_MPL_HAS_XXX_NO_EXPLICIT_TEST_FUNCTION
#     define CROSPI_BOOST_MPL_HAS_MEMBER_REJECT(args, member_macro) \
        template< typename V > \
        static crospi_boost::mpl::aux::no_tag \
        CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)(...); \
      /**/
#   else
#     define CROSPI_BOOST_MPL_HAS_MEMBER_REJECT(args, member_macro) \
        static crospi_boost::mpl::aux::no_tag \
        CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)(...); \
      /**/
#   endif

#   if !CROSPI_BOOST_MPL_HAS_XXX_NO_WRAPPED_TYPES
#     define CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_ACCEPT(z, n, args) \
        template< typename V > \
        static crospi_boost::mpl::aux::yes_tag \
        CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)( \
            crospi_boost::mpl::aux::type_wrapper< V > const volatile* \
          , CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME(args, n) < \
                V::template CROSPI_BOOST_PP_ARRAY_ELEM(1, args) \
            >* = 0 \
        ); \
      /**/
#     define CROSPI_BOOST_MPL_HAS_MEMBER_ACCEPT(args, member_macro) \
        CROSPI_BOOST_PP_REPEAT( \
            CROSPI_BOOST_PP_ARRAY_ELEM(2, args) \
          , CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_ACCEPT \
          , args \
        ) \
      /**/
#   else
#     define CROSPI_BOOST_MPL_HAS_MEMBER_ACCEPT(args, member_macro) \
        template< typename V > \
        static crospi_boost::mpl::aux::yes_tag \
        CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)( \
            V const volatile* \
          , member_macro(args, V, T)* = 0 \
        ); \
      /**/
#   endif

#   if !CROSPI_BOOST_MPL_HAS_XXX_NO_EXPLICIT_TEST_FUNCTION
#     define CROSPI_BOOST_MPL_HAS_MEMBER_TEST(args) \
          sizeof(CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)< U >(0)) \
              == sizeof(crospi_boost::mpl::aux::yes_tag) \
      /**/
#   else
#     if !CROSPI_BOOST_MPL_HAS_XXX_NO_WRAPPED_TYPES
#       define CROSPI_BOOST_MPL_HAS_MEMBER_TEST(args) \
          sizeof( \
              CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)( \
                  static_cast< crospi_boost::mpl::aux::type_wrapper< U >* >(0) \
              ) \
          ) == sizeof(crospi_boost::mpl::aux::yes_tag) \
        /**/
#     else
#       define CROSPI_BOOST_MPL_HAS_MEMBER_TEST(args) \
          sizeof( \
              CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)( \
                  static_cast< U* >(0) \
              ) \
          ) == sizeof(crospi_boost::mpl::aux::yes_tag) \
        /**/
#     endif
#   endif

#   define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECT( \
               args, substitute_macro, member_macro \
           ) \
      template< typename U > \
      struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_NAME(args) { \
          CROSPI_BOOST_MPL_HAS_MEMBER_SUBSTITUTE(args, substitute_macro) \
          CROSPI_BOOST_MPL_HAS_MEMBER_REJECT(args, member_macro) \
          CROSPI_BOOST_MPL_HAS_MEMBER_ACCEPT(args, member_macro) \
          CROSPI_BOOST_STATIC_CONSTANT( \
              bool, value = CROSPI_BOOST_MPL_HAS_MEMBER_TEST(args) \
          ); \
          typedef crospi_boost::mpl::bool_< value > type; \
      }; \
    /**/

#   define CROSPI_BOOST_MPL_HAS_MEMBER_IMPLEMENTATION( \
               args, introspect_macro, substitute_macro, member_macro \
           ) \
      template< \
          typename T \
        , typename fallback_ \
              = crospi_boost::mpl::bool_< CROSPI_BOOST_PP_ARRAY_ELEM(3, args) > \
      > \
      class CROSPI_BOOST_PP_ARRAY_ELEM(0, args) { \
          introspect_macro(args, substitute_macro, member_macro) \
      public: \
          static const bool value \
              = CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_NAME(args)< T >::value; \
          typedef typename CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_NAME(args)< \
              T \
          >::type type; \
      }; \
    /**/

// CROSPI_BOOST_MPL_HAS_MEMBER_WITH_FUNCTION_SFINAE expands to the full
// implementation of the function-based metafunction. Compile with -E
// to see the preprocessor output for this macro.
#   define CROSPI_BOOST_MPL_HAS_MEMBER_WITH_FUNCTION_SFINAE( \
               args, substitute_macro, member_macro \
           ) \
      CROSPI_BOOST_MPL_HAS_MEMBER_IMPLEMENTATION( \
          args \
        , CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECT \
        , substitute_macro \
        , member_macro \
      ) \
    /**/

#   if CROSPI_BOOST_MPL_HAS_XXX_NEEDS_TEMPLATE_SFINAE

#     if !defined(CROSPI_BOOST_MPL_HAS_XXX_NEEDS_NAMESPACE_LEVEL_SUBSTITUTE)
#       if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1400)
#         define CROSPI_BOOST_MPL_HAS_XXX_NEEDS_NAMESPACE_LEVEL_SUBSTITUTE 1
#       endif
#     endif

#     if !CROSPI_BOOST_MPL_HAS_XXX_NEEDS_NAMESPACE_LEVEL_SUBSTITUTE
#       define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME_WITH_TEMPLATE_SFINAE( \
                   args, n \
               ) \
          CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME(args, n) \
        /**/
#     else
#       define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME_WITH_TEMPLATE_SFINAE( \
                   args, n \
               ) \
          CROSPI_BOOST_PP_CAT( \
              boost_mpl_has_xxx_ \
            , CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME(args, n) \
          ) \
        /**/
#     endif

#     define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_TAG_NAME( \
                 args \
             ) \
        CROSPI_BOOST_PP_CAT( \
            CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME_WITH_TEMPLATE_SFINAE( \
                args, 0 \
            ) \
          , _tag \
        ) \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_SUBSTITUTE_WITH_TEMPLATE_SFINAE( \
                 z, n, args \
             ) \
        template< \
             template< CROSPI_BOOST_PP_ENUM_PARAMS_Z(z, CROSPI_BOOST_PP_INC(n), typename U) > class U \
        > \
        struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME_WITH_TEMPLATE_SFINAE( \
                args, n \
               ) { \
            typedef \
                CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_TAG_NAME(args) \
                type; \
        }; \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_SUBSTITUTE_WITH_TEMPLATE_SFINAE( \
                 args, substitute_macro \
             ) \
        typedef void \
            CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_TAG_NAME(args); \
        CROSPI_BOOST_PP_REPEAT( \
            CROSPI_BOOST_PP_ARRAY_ELEM(2, args) \
          , CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_SUBSTITUTE_WITH_TEMPLATE_SFINAE \
          , args \
        ) \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_REJECT_WITH_TEMPLATE_SFINAE( \
                 args, member_macro \
             ) \
        template< \
            typename U \
          , typename V \
                = CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_TAG_NAME(args) \
        > \
        struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args) { \
            CROSPI_BOOST_STATIC_CONSTANT(bool, value = false); \
            typedef crospi_boost::mpl::bool_< value > type; \
        }; \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_ACCEPT_WITH_TEMPLATE_SFINAE( \
                 z, n, args \
             ) \
        template< typename U > \
        struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)< \
            U \
          , typename \
                CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_SUBSTITUTE_NAME_WITH_TEMPLATE_SFINAE( \
                    args, n \
                )< \
                    CROSPI_BOOST_MSVC_TYPENAME U::CROSPI_BOOST_PP_ARRAY_ELEM(1, args)< > \
                >::type \
        > { \
            CROSPI_BOOST_STATIC_CONSTANT(bool, value = true); \
            typedef crospi_boost::mpl::bool_< value > type; \
        }; \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_ACCEPT_WITH_TEMPLATE_SFINAE( \
                 args, member_macro \
             ) \
        CROSPI_BOOST_PP_REPEAT( \
            CROSPI_BOOST_PP_ARRAY_ELEM(2, args) \
          , CROSPI_BOOST_MPL_HAS_MEMBER_MULTI_ACCEPT_WITH_TEMPLATE_SFINAE \
          , args \
        ) \
      /**/

#     define CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECT_WITH_TEMPLATE_SFINAE( \
                 args, substitute_macro, member_macro \
             ) \
        CROSPI_BOOST_MPL_HAS_MEMBER_REJECT_WITH_TEMPLATE_SFINAE(args, member_macro) \
        CROSPI_BOOST_MPL_HAS_MEMBER_ACCEPT_WITH_TEMPLATE_SFINAE(args, member_macro) \
        template< typename U > \
        struct CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_NAME(args) \
            : CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECTION_TEST_NAME(args)< U > { \
        }; \
      /**/
 
// CROSPI_BOOST_MPL_HAS_MEMBER_WITH_TEMPLATE_SFINAE expands to the full
// implementation of the template-based metafunction. Compile with -E
// to see the preprocessor output for this macro.
//
// Note that if CROSPI_BOOST_MPL_HAS_XXX_NEEDS_NAMESPACE_LEVEL_SUBSTITUTE is
// defined CROSPI_BOOST_MPL_HAS_MEMBER_SUBSTITUTE_WITH_TEMPLATE_SFINAE needs
// to be expanded at namespace level before
// CROSPI_BOOST_MPL_HAS_MEMBER_WITH_TEMPLATE_SFINAE can be used.
#     define CROSPI_BOOST_MPL_HAS_MEMBER_WITH_TEMPLATE_SFINAE( \
                 args, substitute_macro, member_macro \
             ) \
        CROSPI_BOOST_MPL_HAS_MEMBER_SUBSTITUTE_WITH_TEMPLATE_SFINAE( \
            args, substitute_macro \
        ) \
        CROSPI_BOOST_MPL_HAS_MEMBER_IMPLEMENTATION( \
            args \
          , CROSPI_BOOST_MPL_HAS_MEMBER_INTROSPECT_WITH_TEMPLATE_SFINAE \
          , substitute_macro \
          , member_macro \
        ) \
      /**/

#   endif // CROSPI_BOOST_MPL_HAS_XXX_NEEDS_TEMPLATE_SFINAE

// Note: In the current implementation the parameter and access macros
// are no longer expanded.
#   if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1400)
#     define CROSPI_BOOST_MPL_HAS_XXX_TEMPLATE_NAMED_DEF(trait, name, default_) \
        CROSPI_BOOST_MPL_HAS_MEMBER_WITH_FUNCTION_SFINAE( \
            ( 4, ( trait, name, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, default_ ) ) \
          , CROSPI_BOOST_MPL_HAS_MEMBER_TEMPLATE_SUBSTITUTE_PARAMETER \
          , CROSPI_BOOST_MPL_HAS_MEMBER_TEMPLATE_ACCESS \
        ) \
      /**/
#   else
#     define CROSPI_BOOST_MPL_HAS_XXX_TEMPLATE_NAMED_DEF(trait, name, default_) \
        CROSPI_BOOST_MPL_HAS_MEMBER_WITH_TEMPLATE_SFINAE( \
            ( 4, ( trait, name, CROSPI_BOOST_MPL_LIMIT_METAFUNCTION_ARITY, default_ ) ) \
          , CROSPI_BOOST_MPL_HAS_MEMBER_TEMPLATE_SUBSTITUTE_PARAMETER \
          , CROSPI_BOOST_MPL_HAS_MEMBER_TEMPLATE_ACCESS \
        ) \
      /**/
#   endif

#else // CROSPI_BOOST_MPL_CFG_NO_HAS_XXX_TEMPLATE

// placeholder implementation

#   define CROSPI_BOOST_MPL_HAS_XXX_TEMPLATE_NAMED_DEF(trait, name, default_) \
      template< typename T \
              , typename fallback_ = crospi_boost::mpl::bool_< default_ > > \
      struct trait { \
          CROSPI_BOOST_STATIC_CONSTANT(bool, value = fallback_::value); \
          typedef fallback_ type; \
      }; \
    /**/

#endif // CROSPI_BOOST_MPL_CFG_NO_HAS_XXX_TEMPLATE

#   define CROSPI_BOOST_MPL_HAS_XXX_TEMPLATE_DEF(name) \
      CROSPI_BOOST_MPL_HAS_XXX_TEMPLATE_NAMED_DEF( \
          CROSPI_BOOST_PP_CAT(has_, name), name, false \
      ) \
    /**/

#endif // CROSPI_BOOST_MPL_HAS_XXX_HPP_INCLUDED
