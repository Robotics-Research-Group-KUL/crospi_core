// Copyright David Abrahams, Daniel Wallin 2003.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETERS_031014_HPP
#define CROSPI_BOOST_PARAMETERS_031014_HPP

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

namespace crospi_boost { namespace parameter { namespace aux {

    // The make_arg_list<> metafunction produces a reversed arg_list,
    // so pass the arguments to the arg_list constructor reversed in turn.
    template <typename ArgList, typename ...Args>
    struct arg_list_factory;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/arg_list.hpp>
#include <utility>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // TODO: Reduce template code bloat. -- Cromwell D. Enage
    template <typename ArgList>
    struct arg_list_factory<ArgList>
    {
        template <typename ...ReversedArgs>
        static inline CROSPI_BOOST_CONSTEXPR ArgList
            reverse(ReversedArgs&&... reversed_args)
        {
            return ArgList(
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_if<
                    ::std::is_same<
#else
                typename ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_same<
#endif
                        typename ArgList::tagged_arg::value_type
                      , ::crospi_boost::parameter::void_
                    >
                  , ::crospi_boost::parameter::aux::value_type_is_void
                  , ::crospi_boost::parameter::aux::value_type_is_not_void
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                >()
#else
                >::type()
#endif
              , ::std::forward<ReversedArgs>(reversed_args)...
            );
        }
    };

    template <typename ArgList, typename A0, typename ...Args>
    struct arg_list_factory<ArgList,A0,Args...>
    {
        template <typename ...ReversedArgs>
        static inline CROSPI_BOOST_CONSTEXPR ArgList
            reverse(A0&& a0, Args&&... args, ReversedArgs&&... reversed_args)
        {
            return ::crospi_boost::parameter::aux
            ::arg_list_factory<ArgList,Args...>::reverse(
                ::std::forward<Args>(args)...
              , ::std::forward<A0>(a0)
              , ::std::forward<ReversedArgs>(reversed_args)...
            );
        }
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/aux_/pack/make_arg_list.hpp>
#include <crospi_boost/parameter/aux_/pack/make_parameter_spec_items.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_keyword_arg.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_template_keyword_arg.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/list.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/pair.hpp>
#include <crospi_boost/mpl/identity.hpp>
#endif

#if !defined(CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
//#include <crospi_boost/mp11/mpl.hpp>
#define CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE ::crospi_boost::mp11::mp_list
#else
#include <crospi_boost/fusion/container/list/list_fwd.hpp>

// Newer versions of MSVC fail on the evaluate_category and
// preprocessor_eval_category test programs when parameters uses
// crospi_boost::fusion::list.
// -- Cromwell D. Enage
#if defined(CROSPI_BOOST_FUSION_HAS_VARIADIC_LIST) && ( \
        !defined(CROSPI_BOOST_MSVC) || (CROSPI_BOOST_MSVC < 1800) \
    )
#include <crospi_boost/fusion/container/list.hpp>
#include <crospi_boost/fusion/mpl.hpp>
#define CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE ::crospi_boost::fusion::list
#else
#include <crospi_boost/fusion/container/deque/deque_fwd.hpp>

#if defined(CROSPI_BOOST_FUSION_HAS_VARIADIC_DEQUE)
#include <crospi_boost/fusion/container/deque.hpp>
#include <crospi_boost/fusion/mpl.hpp>
#define CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE ::crospi_boost::fusion::deque
#else
#include <crospi_boost/mpl/vector.hpp>
#define CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE ::crospi_boost::mpl::vector
#endif  // CROSPI_BOOST_FUSION_HAS_VARIADIC_DEQUE
#endif  // CROSPI_BOOST_FUSION_HAS_VARIADIC_LIST
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE

namespace crospi_boost { namespace parameter {

    template <typename ...Spec>
    struct parameters
    {
        typedef CROSPI_BOOST_PARAMETER_VARIADIC_MPL_SEQUENCE<Spec...> parameter_spec;

        typedef typename ::crospi_boost::parameter::aux
        ::make_deduced_list<parameter_spec>::type deduced_list;

        // If the elements of NamedList match the criteria of overload
        // resolution, returns a type which can be constructed from
        // parameters.  Otherwise, this is not a valid metafunction
        // (no nested ::type).
        template <typename ArgumentPackAndError>
        struct match_base
#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
          : ::crospi_boost::mpl::if_<
                typename ::crospi_boost::parameter::aux::match_parameters_base_cond<
                    ArgumentPackAndError
                  , parameter_spec
                >::type
              , ::crospi_boost::mpl::identity<
                    ::crospi_boost::parameter::parameters<Spec...>
                >
              , ::crospi_boost::parameter::void_
            >
#endif
        {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            using type = ::crospi_boost::mp11::mp_if<
                typename ::crospi_boost::parameter::aux::match_parameters_base_cond<
                    ArgumentPackAndError
                  , parameter_spec
                >::type
              , ::crospi_boost::mp11::mp_identity<
                    ::crospi_boost::parameter::parameters<Spec...>
                >
              , ::crospi_boost::parameter::void_
            >;
#endif
        };

        // Specializations are to be used as an optional argument
        // to eliminate overloads via SFINAE.
        template <typename ...Args>
        struct match
          : ::crospi_boost::parameter::parameters<Spec...>
            ::CROSPI_BOOST_NESTED_TEMPLATE match_base<
                typename ::crospi_boost::parameter::aux::make_arg_list<
                    typename ::crospi_boost::parameter::aux
                    ::make_parameter_spec_items<parameter_spec,Args...>::type
                  , deduced_list
                  , ::crospi_boost::parameter::aux::tag_keyword_arg
                    // Don't emit errors when doing SFINAE.
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                  , ::crospi_boost::mp11::mp_false
#else
                  , ::crospi_boost::mpl::false_
#endif
                >::type
            >::type
        {
        };

        // Metafunction that returns an ArgumentPack.
        template <typename ...Args>
        struct bind
#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
          : ::crospi_boost::mpl::first<
                typename ::crospi_boost::parameter::aux::make_arg_list<
                    typename ::crospi_boost::parameter::aux
                    ::make_parameter_spec_items<parameter_spec,Args...>::type
                  , deduced_list
                  , ::crospi_boost::parameter::aux::tag_template_keyword_arg
                >::type
            >
#endif
        {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            using type = ::crospi_boost::mp11::mp_at_c<
                typename ::crospi_boost::parameter::aux::make_arg_list<
                    typename ::crospi_boost::parameter::aux
                    ::make_parameter_spec_items<parameter_spec,Args...>::type
                  , deduced_list
                  , ::crospi_boost::parameter::aux::tag_template_keyword_arg
                >::type
              , 0
            >;
#endif
        };

        // The function call operator is used to build an arg_list that
        // labels the positional parameters and maintains whatever other
        // tags may have been specified by the caller.
        inline ::crospi_boost::parameter::aux::empty_arg_list operator()() const
        {
            return ::crospi_boost::parameter::aux::empty_arg_list();
        }

        template <typename A0, typename ...Args>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        inline ::crospi_boost::mp11::mp_at_c<
#else
        inline typename ::crospi_boost::mpl::first<
#endif
            typename ::crospi_boost::parameter::aux::make_arg_list<
                typename ::crospi_boost::parameter::aux
                ::make_parameter_spec_items<parameter_spec,A0,Args...>::type
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg
            >::type
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
          , 0
        >
#else
        >::type
#endif
            operator()(A0&& a0, Args&& ...args) const
        {
            typedef typename ::crospi_boost::parameter::aux::make_arg_list<
                typename ::crospi_boost::parameter::aux
                ::make_parameter_spec_items<parameter_spec,A0,Args...>::type
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg
            >::type list_error_pair;

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            using result_type = ::crospi_boost::mp11::mp_at_c<list_error_pair,0>;

            using error = ::crospi_boost::mp11::mp_at_c<list_error_pair,1>;
#else
            typedef typename ::crospi_boost::mpl
            ::first<list_error_pair>::type result_type;

            typedef typename ::crospi_boost::mpl
            ::second<list_error_pair>::type error;
#endif

            error();

            return ::crospi_boost::parameter::aux
            ::arg_list_factory<result_type,A0,Args...>::reverse(
                ::std::forward<A0>(a0)
              , ::std::forward<Args>(args)...
            );
        }
    };
}} // namespace crospi_boost::parameter

#else   // !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/aux_/arg_list.hpp>
#include <crospi_boost/parameter/aux_/pack/make_arg_list.hpp>
#include <crospi_boost/parameter/aux_/pack/make_items.hpp>
#include <crospi_boost/parameter/aux_/pack/make_deduced_items.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_template_keyword_arg.hpp>
#include <crospi_boost/parameter/aux_/preprocessor/binary_seq_for_each.hpp>
#include <crospi_boost/preprocessor/arithmetic/inc.hpp>
#include <crospi_boost/preprocessor/repetition/enum_shifted.hpp>
#include <crospi_boost/preprocessor/repetition/repeat.hpp>
#include <crospi_boost/preprocessor/selection/min.hpp>

#if ( \
        CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY < \
        CROSPI_BOOST_PARAMETER_MAX_ARITY \
    )
#include <crospi_boost/parameter/aux_/pack/tag_keyword_arg_ref.hpp>
#include <crospi_boost/mpl/pair.hpp>
#include <crospi_boost/preprocessor/arithmetic/dec.hpp>
#include <crospi_boost/preprocessor/arithmetic/sub.hpp>
#include <crospi_boost/preprocessor/facilities/intercept.hpp>
#include <crospi_boost/preprocessor/iteration/iterate.hpp>
#include <crospi_boost/preprocessor/repetition/enum.hpp>
#include <crospi_boost/preprocessor/repetition/enum_trailing_params.hpp>
#endif

#if !defined(CROSPI_BOOST_NO_SFINAE) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x592))
#include <crospi_boost/parameter/aux_/pack/tag_keyword_arg.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
#include <crospi_boost/preprocessor/repetition/enum_params.hpp>
#else
#include <crospi_boost/preprocessor/repetition/enum_binary_params.hpp>
#endif

#include <crospi_boost/parameter/aux_/preprocessor/no_perfect_forwarding_begin.hpp>

namespace crospi_boost { namespace parameter {

    template <
        typename PS0
      , CROSPI_BOOST_PP_ENUM_SHIFTED(
            CROSPI_BOOST_PARAMETER_MAX_ARITY
          , CROSPI_BOOST_PARAMETER_template_args
          , PS
        )
    >
    struct parameters
    {
        typedef typename CROSPI_BOOST_PARAMETER_build_deduced_list(
            CROSPI_BOOST_PARAMETER_MAX_ARITY
          , ::crospi_boost::parameter::aux::make_deduced_items
          , PS
        )::type deduced_list;

        // If the elements of NamedList match the criteria of overload
        // resolution, returns a type which can be constructed from
        // parameters.  Otherwise, this is not a valid metafunction
        // (no nested ::type).
#if !defined(CROSPI_BOOST_NO_SFINAE) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x592))
        // If NamedList satisfies the PS0, PS1, ..., this is a metafunction
        // returning parameters.  Otherwise it has no nested ::type.
        template <typename ArgumentPackAndError>
        struct match_base
          : ::crospi_boost::mpl::if_<
                // ::crospi_boost::mpl::and_<
                //    aux::satisfies_requirements_of<NamedList,PS0>
                //  , ::crospi_boost::mpl::and_<
                //       aux::satisfies_requirements_of<NamedList,PS1>...
                //           ..., ::crospi_boost::mpl::true_
                // ...> >
                typename CROSPI_BOOST_PP_REPEAT(
                    CROSPI_BOOST_PARAMETER_MAX_ARITY
                  , CROSPI_BOOST_PARAMETER_satisfies_begin
                  , PS
                )
                ::crospi_boost::is_same<
                    typename ::crospi_boost::mpl
                    ::second<ArgumentPackAndError>::type
                  , ::crospi_boost::parameter::void_
                >
                CROSPI_BOOST_PP_REPEAT(
                    CROSPI_BOOST_PARAMETER_MAX_ARITY
                  , CROSPI_BOOST_PARAMETER_satisfies_end
                  , ::crospi_boost::mpl::false_
                )::type
              , ::crospi_boost::mpl::identity<
                    ::crospi_boost::parameter::parameters<
                        CROSPI_BOOST_PP_ENUM_PARAMS(CROSPI_BOOST_PARAMETER_MAX_ARITY, PS)
                    >
                >
              , ::crospi_boost::parameter::void_
            >
        {
        };
#endif  // SFINAE enabled, not Borland

        // Specializations are to be used as an optional argument
        // to eliminate overloads via SFINAE.
        template <
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
            // Borland simply can't handle default arguments in member
            // class templates.  People wishing to write portable code can
            // explicitly specify CROSPI_BOOST_PARAMETER_MAX_ARITY arguments.
            CROSPI_BOOST_PP_ENUM_PARAMS(CROSPI_BOOST_PARAMETER_MAX_ARITY, typename A)
#else
            CROSPI_BOOST_PP_ENUM_BINARY_PARAMS(
                CROSPI_BOOST_PARAMETER_MAX_ARITY
              , typename A
              , = ::crospi_boost::parameter::void_ CROSPI_BOOST_PP_INTERCEPT
            )
#endif
        >
        struct match
#if !defined(CROSPI_BOOST_NO_SFINAE) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x592))
          : ::crospi_boost::parameter::parameters<
                CROSPI_BOOST_PP_ENUM_PARAMS(CROSPI_BOOST_PARAMETER_MAX_ARITY, PS)
            >::CROSPI_BOOST_NESTED_TEMPLATE match_base<
                typename ::crospi_boost::parameter::aux::make_arg_list<
                    typename CROSPI_BOOST_PARAMETER_build_arg_list(
                        CROSPI_BOOST_PARAMETER_MAX_ARITY
                      , ::crospi_boost::parameter::aux::make_items
                      , PS
                      , A
                    )::type
                  , deduced_list
                  , ::crospi_boost::parameter::aux::tag_keyword_arg
                    // Don't emit errors when doing SFINAE.
                  , ::crospi_boost::mpl::false_
                >::type
            >::type
        {
        };
#else
        {
            typedef ::crospi_boost::parameter::parameters<
                CROSPI_BOOST_PP_ENUM_PARAMS(CROSPI_BOOST_PARAMETER_MAX_ARITY, PS)
            > type;
        };
#endif  // SFINAE enabled, not Borland

        // Metafunction that returns an ArgumentPack.

        // TODO, bind has to instantiate the error type in the result
        // of make_arg_list.

        template <
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
            // Borland simply can't handle default arguments in member
            // class templates.  People wishing to write portable code can
            // explicitly specify CROSPI_BOOST_PARAMETER_MAX_ARITY arguments.
            CROSPI_BOOST_PP_ENUM_PARAMS(CROSPI_BOOST_PARAMETER_MAX_ARITY, typename A)
#else
            CROSPI_BOOST_PP_ENUM_BINARY_PARAMS(
                CROSPI_BOOST_PARAMETER_MAX_ARITY
              , typename A
              , = ::crospi_boost::parameter::void_ CROSPI_BOOST_PP_INTERCEPT
            )
#endif
        >
        struct bind
        {
            typedef typename ::crospi_boost::parameter::aux::make_arg_list<
                typename CROSPI_BOOST_PARAMETER_build_arg_list(
                    CROSPI_BOOST_PARAMETER_MAX_ARITY
                  , ::crospi_boost::parameter::aux::make_items
                  , PS
                  , A
                )::type
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_template_keyword_arg
            >::type result;

            typedef typename ::crospi_boost::mpl::first<result>::type type;
        };

        CROSPI_BOOST_PP_REPEAT(
            CROSPI_BOOST_PARAMETER_MAX_ARITY
          , CROSPI_BOOST_PARAMETER_forward_typedef
          , (PS)(parameter_spec)
        )

        // The function call operator is used to build an arg_list that
        // labels the positional parameters and maintains whatever other
        // tags may have been specified by the caller.
        //
        // !!!NOTE!!!
        //
        // The make_arg_list<> metafunction produces a reversed arg_list,
        // so pass the arguments to the arg_list constructor reversed in turn.
        inline ::crospi_boost::parameter::aux::empty_arg_list operator()() const
        {
            return ::crospi_boost::parameter::aux::empty_arg_list();
        }

#if (0 < CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY)
        CROSPI_BOOST_PP_REPEAT(
            CROSPI_BOOST_PP_MIN(
                CROSPI_BOOST_PP_INC(CROSPI_BOOST_PARAMETER_MAX_ARITY)
              , CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY
            )
          , CROSPI_BOOST_PARAMETER_AUX_PP_BINARY_SEQ_FOR_EACH_Z
          , (CROSPI_BOOST_PARAMETER_function_call_op_overload_R)(_)
        )
#if ( \
        CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY < \
        CROSPI_BOOST_PARAMETER_MAX_ARITY \
    )
#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,( \
        CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY \
      , CROSPI_BOOST_PARAMETER_MAX_ARITY \
      , <crospi_boost/parameter/aux_/preprocessor/overloads.hpp> \
    ))
#include CROSPI_BOOST_PP_ITERATE()
#endif
#else   // (0 == CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY)
        template <typename A0>
        inline typename ::crospi_boost::mpl::first<
            typename ::crospi_boost::parameter::aux::make_arg_list<
                ::crospi_boost::parameter::aux::item<
                    PS0,A0
                >
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg_ref
            >::type
        >::type
            operator()(A0& a0) const
        {
            typedef typename ::crospi_boost::parameter::aux::make_arg_list<
                ::crospi_boost::parameter::aux::item<
                    PS0,A0
                >
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg_ref
            >::type result;

            typedef typename ::crospi_boost::mpl::first<result>::type result_type;
            typedef typename ::crospi_boost::mpl::second<result>::type error;
            error();

            return result_type(
                a0
                // , void_(), void_(), void_() ...
                CROSPI_BOOST_PP_ENUM_TRAILING_PARAMS(
                    CROSPI_BOOST_PP_SUB(CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY, 1)
                  , ::crospi_boost::parameter::aux::void_reference() CROSPI_BOOST_PP_INTERCEPT
                )
            );
        }

        template <typename A0, typename A1>
        inline typename ::crospi_boost::mpl::first<
            typename ::crospi_boost::parameter::aux::make_arg_list<
                ::crospi_boost::parameter::aux::item<
                    PS0,A0
                  , ::crospi_boost::parameter::aux::item<
                        PS1,A1
                    >
                >
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg_ref
            >::type
        >::type
            operator()(A0& a0, A1& a1) const
        {
            typedef typename ::crospi_boost::parameter::aux::make_arg_list<
                ::crospi_boost::parameter::aux::item<
                    PS0,A0
                  , ::crospi_boost::parameter::aux::item<
                        PS1,A1
                    >
                >
              , deduced_list
              , ::crospi_boost::parameter::aux::tag_keyword_arg
            >::type result;

            typedef typename ::crospi_boost::mpl::first<result>::type result_type;
            typedef typename ::crospi_boost::mpl::second<result>::type error;
            error();

            return result_type(
                a1
              , a0
                // , void_(), void_() ...
                CROSPI_BOOST_PP_ENUM_TRAILING_PARAMS(
                    CROSPI_BOOST_PP_SUB(CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY, 2)
                  , ::crospi_boost::parameter::aux::void_reference() CROSPI_BOOST_PP_INTERCEPT
                )
            );
        }

#if (2 < CROSPI_BOOST_PARAMETER_MAX_ARITY)
        // Higher arities are handled by the preprocessor
#define CROSPI_BOOST_PP_ITERATION_PARAMS_1 \
    (3,( \
        3 \
      , CROSPI_BOOST_PARAMETER_MAX_ARITY \
      , <crospi_boost/parameter/aux_/preprocessor/overloads.hpp> \
    ))
#include CROSPI_BOOST_PP_ITERATE()
#endif
#endif  // exponential overloads
    };
}} // namespace crospi_boost::parameter

#include <crospi_boost/parameter/aux_/preprocessor/no_perfect_forwarding_end.hpp>

#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING
#endif  // include guard

