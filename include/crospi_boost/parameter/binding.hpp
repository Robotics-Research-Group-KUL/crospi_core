// Copyright David Abrahams 2005.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_BINDING_DWA200558_HPP
#define CROSPI_BOOST_PARAMETER_BINDING_DWA200558_HPP

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/list.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/assert.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

namespace crospi_boost { namespace parameter {

    // A metafunction that, given an argument pack, returns the reference type
    // of the parameter identified by the given keyword.  If no such parameter
    // has been specified, returns Default

    template <typename Parameters, typename Keyword, typename Default>
    struct binding0
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = ::crospi_boost::mp11::mp_apply_q<
            typename Parameters::binding
          , ::crospi_boost::mp11::mp_list<Keyword,Default,::crospi_boost::mp11::mp_true>
        >;

        static_assert(
            ::crospi_boost::mp11::mp_if<
                ::std::is_same<Default,::crospi_boost::parameter::void_>
              , ::crospi_boost::mp11::mp_if<
                    ::std::is_same<type,::crospi_boost::parameter::void_>
                  , ::crospi_boost::mp11::mp_false
                  , ::crospi_boost::mp11::mp_true
                >
              , ::crospi_boost::mp11::mp_true
            >::value
          , "required parameters must not result in void_ type"
        );
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        typedef typename ::crospi_boost::mpl::apply_wrap3<
            typename Parameters::binding
          , Keyword
          , Default
          , ::crospi_boost::mpl::true_
        >::type type;

        CROSPI_BOOST_MPL_ASSERT((
            typename ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_same<Default,::crospi_boost::parameter::void_>
              , ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_same<type,::crospi_boost::parameter::void_>
                  , ::crospi_boost::mpl::false_
                  , ::crospi_boost::mpl::true_
                >
              , ::crospi_boost::mpl::true_
            >::type
        ));
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
    };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    template <typename Placeholder, typename Keyword, typename Default>
    struct binding1
    {
        using type = ::crospi_boost::mp11::mp_apply_q<
            Placeholder
          , ::crospi_boost::mp11::mp_list<Keyword,Default,::crospi_boost::mp11::mp_true>
        >;

        static_assert(
            ::crospi_boost::mp11::mp_if<
                ::std::is_same<Default,::crospi_boost::parameter::void_>
              , ::crospi_boost::mp11::mp_if<
                    ::std::is_same<type,::crospi_boost::parameter::void_>
                  , ::crospi_boost::mp11::mp_false
                  , ::crospi_boost::mp11::mp_true
                >
              , ::crospi_boost::mp11::mp_true
            >::value
          , "required parameters must not result in void_ type"
        );
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}} // namespace crospi_boost::parameter

#include <crospi_boost/parameter/aux_/is_placeholder.hpp>

namespace crospi_boost { namespace parameter {

    template <
        typename Parameters
      , typename Keyword
      , typename Default = ::crospi_boost::parameter::void_
    >
    struct binding
#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::parameter::aux::is_mpl_placeholder<Parameters>
          , ::crospi_boost::mpl::identity<int>
          , ::crospi_boost::parameter::binding0<Parameters,Keyword,Default>
        >
#endif
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = typename ::crospi_boost::mp11::mp_if<
            ::crospi_boost::parameter::aux::is_mpl_placeholder<Parameters>
          , ::crospi_boost::mp11::mp_identity<int>
          , ::crospi_boost::mp11::mp_if<
                ::crospi_boost::parameter::aux::is_mp11_placeholder<Parameters>
              , ::crospi_boost::parameter::binding1<Parameters,Keyword,Default>
              , ::crospi_boost::parameter::binding0<Parameters,Keyword,Default>
            >
        >::type;
#endif
    };
}} // namespace crospi_boost::parameter

#include <crospi_boost/parameter/aux_/result_of0.hpp>

namespace crospi_boost { namespace parameter {

    // A metafunction that, given an argument pack, returns the reference type
    // of the parameter identified by the given keyword.  If no such parameter
    // has been specified, returns the type returned by invoking DefaultFn
    template <typename Parameters, typename Keyword, typename DefaultFn>
    struct lazy_binding
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = ::crospi_boost::mp11::mp_apply_q<
            typename Parameters::binding
          , ::crospi_boost::mp11::mp_list<
                Keyword
              , typename ::crospi_boost::parameter::aux::result_of0<DefaultFn>::type
              , ::crospi_boost::mp11::mp_true
            >
        >;
#else
        typedef typename ::crospi_boost::mpl::apply_wrap3<
            typename Parameters::binding
          , Keyword
          , typename ::crospi_boost::parameter::aux::result_of0<DefaultFn>::type
          , ::crospi_boost::mpl::true_
        >::type type;
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
    };
}} // namespace crospi_boost::parameter

#endif  // include guard

