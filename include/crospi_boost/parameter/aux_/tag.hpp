// Copyright David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_TAG_DWA2005610_HPP
#define CROSPI_BOOST_PARAMETER_AUX_TAG_DWA2005610_HPP

#include <crospi_boost/parameter/aux_/unwrap_cv_reference.hpp>
#include <crospi_boost/parameter/aux_/tagged_argument.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, >= 1910)
// MSVC-14.1+ assigns rvalue references to tagged_argument instances
// instead of tagged_argument_rref instances with this code.
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Arg>
    struct tag_if_lvalue_reference
    {
        using type = ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
            ::crospi_boost::parameter::aux::tagged_argument<
                Keyword
              , typename ::crospi_boost::parameter::aux
                ::unwrap_cv_reference<Arg>::type
            >
        >;
    };

    template <typename Keyword, typename Arg>
    struct tag_if_scalar
    {
        using type = ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
            ::crospi_boost::parameter::aux
            ::tagged_argument<Keyword,typename ::std::add_const<Arg>::type>
        >;
    };

    template <typename Keyword, typename Arg>
    using tag_if_otherwise = ::crospi_boost::mp11::mp_if<
        ::std::is_scalar<typename ::std::remove_const<Arg>::type>
      , ::crospi_boost::parameter::aux::tag_if_scalar<Keyword,Arg>
      , ::crospi_boost::mp11::mp_identity<
            ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
                ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
            >
        >
    >;

    template <typename Keyword, typename Arg>
    using tag = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_if<
            ::std::is_lvalue_reference<Arg>
          , ::crospi_boost::mp11::mp_true
          , ::crospi_boost::parameter::aux::is_cv_reference_wrapper<Arg>
        >
      , ::crospi_boost::parameter::aux::tag_if_lvalue_reference<Keyword,Arg>
      , ::crospi_boost::parameter::aux::tag_if_otherwise<Keyword,Arg>
    >;
}}} // namespace crospi_boost::parameter::aux_

#elif defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/type_traits/add_const.hpp>
#include <crospi_boost/type_traits/is_scalar.hpp>
#include <crospi_boost/type_traits/is_lvalue_reference.hpp>
#include <crospi_boost/type_traits/remove_const.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename ActualArg>
    struct tag
    {
        typedef typename ::crospi_boost::parameter::aux
        ::unwrap_cv_reference<ActualArg>::type Arg;
        typedef typename ::crospi_boost::add_const<Arg>::type ConstArg;
        typedef typename ::crospi_boost::remove_const<Arg>::type MutArg;
        typedef typename ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::is_lvalue_reference<ActualArg>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::parameter::aux::is_cv_reference_wrapper<ActualArg>
            >::type
          , ::crospi_boost::mpl::identity<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
#endif
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                >
#endif
            >
          , ::crospi_boost::mpl::if_<
                ::crospi_boost::is_scalar<MutArg>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
              , ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword,ConstArg>
                >
              , ::crospi_boost::parameter::aux::tagged_argument_list_of_1<
                    ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
                >
#else
              , ::crospi_boost::parameter::aux::tagged_argument<Keyword,ConstArg>
              , ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
#endif
            >
        >::type type;
    };
}}} // namespace crospi_boost::parameter::aux_

#else   // !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

namespace crospi_boost { namespace parameter { namespace aux {

    template <
        typename Keyword
      , typename Arg
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
      , typename = typename ::crospi_boost::parameter::aux
        ::is_cv_reference_wrapper<Arg>::type
#endif
    >
    struct tag
    {
        typedef ::crospi_boost::parameter::aux::tagged_argument<
            Keyword
          , typename ::crospi_boost::parameter::aux::unwrap_cv_reference<Arg>::type
        > type;
    };
}}} // namespace crospi_boost::parameter::aux_

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Arg>
    struct tag<Keyword,Arg,::crospi_boost::mpl::false_>
    {
        typedef ::crospi_boost::parameter::aux::tagged_argument<
            Keyword
          , typename ::crospi_boost::remove_reference<Arg>::type
        > type;
    };
}}} // namespace crospi_boost::parameter::aux_

#endif  // Borland workarounds needed.
#endif  // MP11 or perfect forwarding support
#endif  // include guard

