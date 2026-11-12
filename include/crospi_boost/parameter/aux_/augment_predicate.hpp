// Copyright Cromwell D. Enage 2018.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUGMENT_PREDICATE_HPP
#define CROSPI_BOOST_PARAMETER_AUGMENT_PREDICATE_HPP

#include <crospi_boost/parameter/keyword_fwd.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/type_traits/is_lvalue_reference.hpp>
#include <crospi_boost/type_traits/is_scalar.hpp>
#include <crospi_boost/type_traits/is_same.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename V, typename R, typename Tag>
    struct augment_predicate_check_consume_ref
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_scalar<V>
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_same<
                    typename Tag::qualifier
                  , ::crospi_boost::parameter::consume_reference
                >
              , ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_lvalue_reference<R>
                  , ::crospi_boost::mpl::false_
                  , ::crospi_boost::mpl::true_
                >
              , crospi_boost::mpl::true_
            >
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/type_traits/is_const.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename V, typename R, typename Tag>
    struct augment_predicate_check_out_ref
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<
                typename Tag::qualifier
              , ::crospi_boost::parameter::out_reference
            >
          , ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_lvalue_reference<R>
              , ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_const<V>
                  , ::crospi_boost::mpl::false_
                  , ::crospi_boost::mpl::true_
                >
              , ::crospi_boost::mpl::false_
            >
          , ::crospi_boost::mpl::true_
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/lambda_tag.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/lambda.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <
        typename Predicate
      , typename R
      , typename Tag
      , typename T
      , typename Args
    >
    class augment_predicate
    {
        typedef typename ::crospi_boost::mpl::lambda<
            Predicate
          , ::crospi_boost::parameter::aux::lambda_tag
        >::type _actual_predicate;

     public:
        typedef typename ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::parameter::aux
                ::augment_predicate_check_consume_ref<T,R,Tag>
              , ::crospi_boost::parameter::aux
                ::augment_predicate_check_out_ref<T,R,Tag>
              , ::crospi_boost::mpl::false_
            >::type
          , ::crospi_boost::mpl::apply_wrap2<_actual_predicate,T,Args>
          , ::crospi_boost::mpl::false_
        >::type type;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename V, typename R, typename Tag>
    using augment_predicate_check_consume_ref_mp11 = ::crospi_boost::mp11::mp_if<
        ::std::is_scalar<V>
      , ::crospi_boost::mp11::mp_true
      , ::crospi_boost::mp11::mp_if<
            ::std::is_same<
                typename Tag::qualifier
              , ::crospi_boost::parameter::consume_reference
            >
          , ::crospi_boost::mp11::mp_if<
                ::std::is_lvalue_reference<R>
              , ::crospi_boost::mp11::mp_false
              , ::crospi_boost::mp11::mp_true
            >
          , crospi_boost::mp11::mp_true
        >
    >;

    template <typename V, typename R, typename Tag>
    using augment_predicate_check_out_ref_mp11 = ::crospi_boost::mp11::mp_if<
        ::std::is_same<
            typename Tag::qualifier
          , ::crospi_boost::parameter::out_reference
        >
      , ::crospi_boost::mp11::mp_if<
            ::std::is_lvalue_reference<R>
          , ::crospi_boost::mp11::mp_if<
                ::std::is_const<V>
              , ::crospi_boost::mp11::mp_false
              , ::crospi_boost::mp11::mp_true
            >
          , ::crospi_boost::mp11::mp_false
        >
      , ::crospi_boost::mp11::mp_true
    >;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mp11/list.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <
        typename Predicate
      , typename R
      , typename Tag
      , typename T
      , typename Args
    >
    struct augment_predicate_mp11_impl
    {
        using type = ::crospi_boost::mp11::mp_if<
            ::crospi_boost::mp11::mp_if<
                ::crospi_boost::parameter::aux
                ::augment_predicate_check_consume_ref_mp11<T,R,Tag>
              , ::crospi_boost::parameter::aux
                ::augment_predicate_check_out_ref_mp11<T,R,Tag>
              , ::crospi_boost::mp11::mp_false
            >
          , ::crospi_boost::mp11
            ::mp_apply_q<Predicate,::crospi_boost::mp11::mp_list<T,Args> >
          , ::crospi_boost::mp11::mp_false
        >;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/has_nested_template_fn.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <
        typename Predicate
      , typename R
      , typename Tag
      , typename T
      , typename Args
    >
    using augment_predicate_mp11 = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::parameter::aux::has_nested_template_fn<Predicate>
      , ::crospi_boost::parameter::aux
        ::augment_predicate_mp11_impl<Predicate,R,Tag,T,Args>
      , ::crospi_boost::parameter::aux
        ::augment_predicate<Predicate,R,Tag,T,Args>
    >;
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

