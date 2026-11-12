
#ifndef CROSPI_BOOST_MPL_ASSERT_HPP_INCLUDED
#define CROSPI_BOOST_MPL_ASSERT_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2006
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/not.hpp>
#include <crospi_boost/mpl/aux_/value_wknd.hpp>
#include <crospi_boost/mpl/aux_/nested_type_wknd.hpp>
#include <crospi_boost/mpl/aux_/yes_no.hpp>
#include <crospi_boost/mpl/aux_/na.hpp>
#include <crospi_boost/mpl/aux_/adl_barrier.hpp>

#include <crospi_boost/mpl/aux_/config/nttp.hpp>
#include <crospi_boost/mpl/aux_/config/dtp.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/msvc.hpp>
#include <crospi_boost/mpl/aux_/config/gpu.hpp>
#include <crospi_boost/mpl/aux_/config/static_constant.hpp>
#include <crospi_boost/mpl/aux_/config/pp_counter.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#include <crospi_boost/preprocessor/cat.hpp>

#include <crospi_boost/config.hpp> // make sure 'size_t' is placed into 'std'
#include <cstddef>

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1700)
#include <crospi_boost/mpl/if.hpp>
#endif

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
    || (CROSPI_BOOST_MPL_CFG_GCC != 0) \
    || CROSPI_BOOST_WORKAROUND(__IBMCPP__, <= 600)
#   define CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES
#endif

#if CROSPI_BOOST_WORKAROUND(__MWERKS__, < 0x3202) \
    || CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 238) \
    || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
    || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
#   define CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER
#endif

// agurt, 10/nov/06: use enums for Borland (which cannot cope with static constants) 
// and GCC/PGI/Clang (which issues "unused variable" warnings when static constants are used 
// at a function scope)
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
    || (CROSPI_BOOST_MPL_CFG_GCC != 0) || (CROSPI_BOOST_MPL_CFG_GPU != 0) || defined(__PGI) || defined(__clang__)
#   define CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT(T, expr) enum { expr }
#else
#   define CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT(T, expr) CROSPI_BOOST_STATIC_CONSTANT(T, expr)
#endif


CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN

struct failed {};

// agurt, 24/aug/04: MSVC 7.1 workaround here and below: return/accept 
// 'assert<false>' by reference; can't apply it unconditionally -- apparently it
// degrades the quality of GCC diagnostics
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
#   define AUX778076_ASSERT_ARG(x) x&
#else
#   define AUX778076_ASSERT_ARG(x) x
#endif

template< bool C >  struct assert        { typedef void* type; };
template<>          struct assert<false> { typedef AUX778076_ASSERT_ARG(assert) type; };

template< bool C >
int assertion_failed( typename assert<C>::type );

template< bool C >
struct assertion
{
    static int failed( assert<false> );
};

template<>
struct assertion<true>
{
    static int failed( void* );
};

struct assert_
{
#if !defined(CROSPI_BOOST_MPL_CFG_NO_DEFAULT_PARAMETERS_IN_NESTED_TEMPLATES)
    template< typename T1, typename T2 = na, typename T3 = na, typename T4 = na > struct types {};
#endif
    static assert_ const arg;
    enum relations { equal = 1, not_equal, greater, greater_equal, less, less_equal };
};


#if !defined(CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES)

bool operator==( failed, failed );
bool operator!=( failed, failed );
bool operator>( failed, failed );
bool operator>=( failed, failed );
bool operator<( failed, failed );
bool operator<=( failed, failed );

#if defined(__EDG_VERSION__)
template< bool (*)(failed, failed), long x, long y > struct assert_relation {};
#   define CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x, y, r) assert_relation<r,x,y>
#else
template< CROSPI_BOOST_MPL_AUX_NTTP_DECL(long, x), CROSPI_BOOST_MPL_AUX_NTTP_DECL(long, y), bool (*)(failed, failed) > 
struct assert_relation {};
#   define CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x, y, r) assert_relation<x,y,r>
#endif

#else // CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES

crospi_boost::mpl::aux::weighted_tag<1>::type operator==( assert_, assert_ );
crospi_boost::mpl::aux::weighted_tag<2>::type operator!=( assert_, assert_ );
crospi_boost::mpl::aux::weighted_tag<3>::type operator>(  assert_, assert_ );
crospi_boost::mpl::aux::weighted_tag<4>::type operator>=( assert_, assert_ );
crospi_boost::mpl::aux::weighted_tag<5>::type operator<( assert_, assert_ );
crospi_boost::mpl::aux::weighted_tag<6>::type operator<=( assert_, assert_ );

template< assert_::relations r, long x, long y > struct assert_relation {};

#endif 

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1700)

template<class Pred>
struct extract_assert_pred;

template<class Pred>
struct extract_assert_pred<void(Pred)> { typedef Pred type; };

template<class Pred>
struct eval_assert {
    typedef typename extract_assert_pred<Pred>::type P;
    typedef typename P::type p_type;
    typedef typename ::crospi_boost::mpl::if_c<p_type::value,
        AUX778076_ASSERT_ARG(assert<false>),
        failed ************ P::************
    >::type type;
};

template<class Pred>
struct eval_assert_not {
    typedef typename extract_assert_pred<Pred>::type P;
    typedef typename P::type p_type;
    typedef typename ::crospi_boost::mpl::if_c<!p_type::value,
        AUX778076_ASSERT_ARG(assert<false>),
        failed ************ ::crospi_boost::mpl::not_<P>::************
    >::type type;
};

template< typename T >
T make_assert_arg();

#elif !defined(CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER)

template< bool > struct assert_arg_pred_impl { typedef int type; };
template<> struct assert_arg_pred_impl<true> { typedef void* type; };

template< typename P > struct assert_arg_pred
{
    typedef typename P::type p_type;
    typedef typename assert_arg_pred_impl< p_type::value >::type type;
};

template< typename P > struct assert_arg_pred_not
{
    typedef typename P::type p_type;
    CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( bool, p = !p_type::value );
    typedef typename assert_arg_pred_impl<p>::type type;
};

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC_VERSION, >= 80000)
#define CROSPI_BOOST_MPL_IGNORE_PARENTHESES_WARNING
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wparentheses"
#endif

template< typename Pred >
failed ************ (Pred::************
      assert_arg( void (*)(Pred), typename assert_arg_pred<Pred>::type )
    );

template< typename Pred >
failed ************ (crospi_boost::mpl::not_<Pred>::************
      assert_not_arg( void (*)(Pred), typename assert_arg_pred_not<Pred>::type )
    );

#ifdef CROSPI_BOOST_MPL_IGNORE_PARENTHESES_WARNING
#undef CROSPI_BOOST_MPL_IGNORE_PARENTHESES_WARNING
#pragma GCC diagnostic pop
#endif

template< typename Pred >
AUX778076_ASSERT_ARG(assert<false>)
assert_arg( void (*)(Pred), typename assert_arg_pred_not<Pred>::type );

template< typename Pred >
AUX778076_ASSERT_ARG(assert<false>)
assert_not_arg( void (*)(Pred), typename assert_arg_pred<Pred>::type );


#else // CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER
        
template< bool c, typename Pred > struct assert_arg_type_impl
{
    typedef failed      ************ Pred::* mwcw83_wknd;
    typedef mwcw83_wknd ************* type;
};

template< typename Pred > struct assert_arg_type_impl<true,Pred>
{
    typedef AUX778076_ASSERT_ARG(assert<false>) type;
};

template< typename Pred > struct assert_arg_type
    : assert_arg_type_impl< CROSPI_BOOST_MPL_AUX_VALUE_WKND(CROSPI_BOOST_MPL_AUX_NESTED_TYPE_WKND(Pred))::value, Pred >
{
};

template< typename Pred >
typename assert_arg_type<Pred>::type 
assert_arg(void (*)(Pred), int);

template< typename Pred >
typename assert_arg_type< crospi_boost::mpl::not_<Pred> >::type 
assert_not_arg(void (*)(Pred), int);

#   if !defined(CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES)
template< long x, long y, bool (*r)(failed, failed) >
typename assert_arg_type_impl< false,CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x,y,r) >::type
assert_rel_arg( CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x,y,r) );
#   else
template< assert_::relations r, long x, long y >
typename assert_arg_type_impl< false,assert_relation<r,x,y> >::type
assert_rel_arg( assert_relation<r,x,y> );
#   endif

#endif // CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER

#undef AUX778076_ASSERT_ARG

CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1700)

// CROSPI_BOOST_MPL_ASSERT((pred<x,...>))

#define CROSPI_BOOST_MPL_ASSERT(pred) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
          crospi_boost::mpl::assertion_failed<false>( \
              crospi_boost::mpl::make_assert_arg< \
                  typename crospi_boost::mpl::eval_assert<void pred>::type \
                >() \
            ) \
        ) \
    ) \
/**/

// CROSPI_BOOST_MPL_ASSERT_NOT((pred<x,...>))

#define CROSPI_BOOST_MPL_ASSERT_NOT(pred) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
          crospi_boost::mpl::assertion_failed<false>( \
              crospi_boost::mpl::make_assert_arg< \
                  typename crospi_boost::mpl::eval_assert_not<void pred>::type \
                >() \
            ) \
        ) \
    ) \
/**/

#else

// CROSPI_BOOST_MPL_ASSERT((pred<x,...>))

#define CROSPI_BOOST_MPL_ASSERT(pred) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
          crospi_boost::mpl::assertion_failed<false>( \
              crospi_boost::mpl::assert_arg( (void (*) pred)CROSPI_BOOST_NULLPTR, 1 ) \
            ) \
        ) \
    ) \
/**/

// CROSPI_BOOST_MPL_ASSERT_NOT((pred<x,...>))

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1300)
#   define CROSPI_BOOST_MPL_ASSERT_NOT(pred) \
enum { \
      CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
          crospi_boost::mpl::assertion<false>::failed( \
              crospi_boost::mpl::assert_not_arg( (void (*) pred)CROSPI_BOOST_NULLPTR, 1 ) \
            ) \
        ) \
}\
/**/
#else
#   define CROSPI_BOOST_MPL_ASSERT_NOT(pred) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
          crospi_boost::mpl::assertion_failed<false>( \
              crospi_boost::mpl::assert_not_arg( (void (*) pred)CROSPI_BOOST_NULLPTR, 1 ) \
            ) \
        ) \
   ) \
/**/
#endif

#endif

// CROSPI_BOOST_MPL_ASSERT_RELATION(x, ==|!=|<=|<|>=|>, y)

#if defined(CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES)

#   if !defined(CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER)
// agurt, 9/nov/06: 'enum' below is a workaround for gcc 4.0.4/4.1.1 bugs #29522 and #29518
#   define CROSPI_BOOST_MPL_ASSERT_RELATION_IMPL(counter, x, rel, y)      \
enum { CROSPI_BOOST_PP_CAT(mpl_assert_rel_value,counter) = (x rel y) }; \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,counter) = sizeof( \
        crospi_boost::mpl::assertion_failed<CROSPI_BOOST_PP_CAT(mpl_assert_rel_value,counter)>( \
            (crospi_boost::mpl::failed ************ ( crospi_boost::mpl::assert_relation< \
                  crospi_boost::mpl::assert_::relations( sizeof( \
                      crospi_boost::mpl::assert_::arg rel crospi_boost::mpl::assert_::arg \
                    ) ) \
                , x \
                , y \
                >::************)) 0 ) \
        ) \
    ) \
/**/
#   else
#   define CROSPI_BOOST_MPL_ASSERT_RELATION_IMPL(counter, x, rel, y)    \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assert_rel,counter) = sizeof( \
          crospi_boost::mpl::assert_::arg rel crospi_boost::mpl::assert_::arg \
        ) \
    ); \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( bool, CROSPI_BOOST_PP_CAT(mpl_assert_rel_value,counter) = (x rel y) ); \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,counter) = sizeof( \
        crospi_boost::mpl::assertion_failed<CROSPI_BOOST_PP_CAT(mpl_assert_rel_value,counter)>( \
              crospi_boost::mpl::assert_rel_arg( crospi_boost::mpl::assert_relation< \
                  crospi_boost::mpl::assert_::relations(CROSPI_BOOST_PP_CAT(mpl_assert_rel,counter)) \
                , x \
                , y \
                >() ) \
            ) \
        ) \
    ) \
/**/
#   endif

#   define CROSPI_BOOST_MPL_ASSERT_RELATION(x, rel, y) \
CROSPI_BOOST_MPL_ASSERT_RELATION_IMPL(CROSPI_BOOST_MPL_AUX_PP_COUNTER(), x, rel, y) \
/**/

#else // !CROSPI_BOOST_MPL_CFG_ASSERT_USE_RELATION_NAMES

#   if defined(CROSPI_BOOST_MPL_CFG_ASSERT_BROKEN_POINTER_TO_POINTER_TO_MEMBER)
#   define CROSPI_BOOST_MPL_ASSERT_RELATION(x, rel, y) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
        crospi_boost::mpl::assertion_failed<(x rel y)>( crospi_boost::mpl::assert_rel_arg( \
              crospi_boost::mpl::CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x,y,(&crospi_boost::mpl::operator rel))() \
            ) ) \
        ) \
    ) \
/**/
#   else
#   define CROSPI_BOOST_MPL_ASSERT_RELATION(x, rel, y) \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,CROSPI_BOOST_MPL_AUX_PP_COUNTER()) = sizeof( \
        crospi_boost::mpl::assertion_failed<(x rel y)>( (crospi_boost::mpl::failed ************ ( \
            crospi_boost::mpl::CROSPI_BOOST_MPL_AUX_ASSERT_RELATION(x,y,(&crospi_boost::mpl::operator rel))::************))0 ) \
        ) \
    ) \
/**/
#   endif

#endif


// CROSPI_BOOST_MPL_ASSERT_MSG( (pred<x,...>::value), USER_PROVIDED_MESSAGE, (types<x,...>) ) 

#if CROSPI_BOOST_WORKAROUND(__MWERKS__, CROSPI_BOOST_TESTED_AT(0x3202))
#   define CROSPI_BOOST_MPL_ASSERT_MSG_IMPL( counter, c, msg, types_ ) \
struct msg; \
typedef struct CROSPI_BOOST_PP_CAT(msg,counter) : crospi_boost::mpl::assert_ \
{ \
    using crospi_boost::mpl::assert_::types; \
    static crospi_boost::mpl::failed ************ (msg::************ assert_arg()) types_ \
    { return 0; } \
} CROSPI_BOOST_PP_CAT(mpl_assert_arg,counter); \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,counter) = sizeof( \
        crospi_boost::mpl::assertion<(c)>::failed( CROSPI_BOOST_PP_CAT(mpl_assert_arg,counter)::assert_arg() ) \
        ) \
    ) \
/**/
#else
#   define CROSPI_BOOST_MPL_ASSERT_MSG_IMPL( counter, c, msg, types_ )  \
struct msg; \
typedef struct CROSPI_BOOST_PP_CAT(msg,counter) : crospi_boost::mpl::assert_ \
{ \
    static crospi_boost::mpl::failed ************ (msg::************ assert_arg()) types_ \
    { return 0; } \
} CROSPI_BOOST_PP_CAT(mpl_assert_arg,counter); \
CROSPI_BOOST_MPL_AUX_ASSERT_CONSTANT( \
      std::size_t \
    , CROSPI_BOOST_PP_CAT(mpl_assertion_in_line_,counter) = sizeof( \
        crospi_boost::mpl::assertion_failed<(c)>( CROSPI_BOOST_PP_CAT(mpl_assert_arg,counter)::assert_arg() ) \
        ) \
    ) \
/**/
#endif

#if 0
// Work around CROSPI_BOOST_MPL_ASSERT_MSG_IMPL generating multiple definition linker errors on VC++8.
// #if defined(CROSPI_BOOST_MSVC) && CROSPI_BOOST_MSVC < 1500
#   include <crospi_boost/static_assert.hpp>
#   define CROSPI_BOOST_MPL_ASSERT_MSG( c, msg, types_ ) \
CROSPI_BOOST_STATIC_ASSERT_MSG( c, #msg ) \
/**/
#else
#   define CROSPI_BOOST_MPL_ASSERT_MSG( c, msg, types_ ) \
CROSPI_BOOST_MPL_ASSERT_MSG_IMPL( CROSPI_BOOST_MPL_AUX_PP_COUNTER(), c, msg, types_ ) \
/**/
#endif

#endif // CROSPI_BOOST_MPL_ASSERT_HPP_INCLUDED
