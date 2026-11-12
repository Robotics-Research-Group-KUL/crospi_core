// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_SATISFIES_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_SATISFIES_HPP

#include <crospi_boost/parameter/config.hpp>

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
#include <crospi_boost/parameter/aux_/arg_list.hpp>
#include <crospi_boost/parameter/aux_/augment_predicate.hpp>
#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#else   // !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#endif
#include <crospi_boost/parameter/aux_/yesno.hpp>
#include <crospi_boost/parameter/aux_/preprocessor/nullptr.hpp>
#endif  // MSVC-7.1 workarounds needed

namespace crospi_boost { namespace parameter { namespace aux {

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
    template <typename ArgList, typename ParameterRequirements, typename Bound>
    struct satisfies_impl
      : ::crospi_boost::parameter::aux::augment_predicate<
            typename ParameterRequirements::predicate
          , typename ArgList::reference
          , typename ArgList::key_type
          , Bound
          , ArgList
        >
    {
    };
#endif

    // Returns mpl::true_ iff the given ParameterRequirements are satisfied by
    // ArgList.
    template <typename ArgList, typename ParameterRequirements>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using satisfies = ::crospi_boost::mp11::mp_bool<
        sizeof(
            ::crospi_boost::parameter::aux::to_yesno(
                ArgList::satisfies(
                    static_cast<ParameterRequirements*>(
                        CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR
                    )
                  , static_cast<ArgList*>(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                )
            )
        ) == sizeof(::crospi_boost::parameter::aux::yes_tag)
    >;
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    class satisfies
    {
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
        // VC7.1 can't handle the sizeof() implementation below,
        // so we use this instead.
        typedef typename ::crospi_boost::mpl::apply_wrap3<
            typename ArgList::binding
          , typename ParameterRequirements::keyword
          , ::crospi_boost::parameter::void_
          , ::crospi_boost::mpl::false_
        >::type _bound;

     public:
        typedef typename ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<_bound,::crospi_boost::parameter::void_>
          , typename ParameterRequirements::has_default
          , ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_same<
                    ArgList
                  , ::crospi_boost::parameter::aux::empty_arg_list
                >
              , ::crospi_boost::mpl::false_
              , ::crospi_boost::parameter::aux::satisfies_impl<
                    ArgList
                  , ParameterRequirements
                  , _bound
                >
            >
        >::type type;
#else   // !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1310)
        CROSPI_BOOST_STATIC_CONSTANT(
            bool, _value = (
                sizeof(
                    ::crospi_boost::parameter::aux::to_yesno(
                        ArgList::satisfies(
                            static_cast<ParameterRequirements*>(
                                CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR
                            )
                          , static_cast<ArgList*>(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                        )
                    )
                ) == sizeof(::crospi_boost::parameter::aux::yes_tag)
            )
        );

     public:
        typedef ::crospi_boost::mpl::bool_<
            ::crospi_boost::parameter::aux
            ::satisfies<ArgList,ParameterRequirements>::_value
        > type;
#endif  // MSVC-7.1 workarounds needed
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/pack/as_parameter_requirements.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // Returns mpl::true_ if the requirements of the given ParameterSpec
    // are satisfied by ArgList.
    template <typename ArgList, typename ParameterSpec>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using satisfies_requirements_of = ::crospi_boost::parameter::aux::satisfies<
        ArgList
      , typename ::crospi_boost::parameter::aux
        ::as_parameter_requirements<ParameterSpec>::type
    >;
#else
    struct satisfies_requirements_of
      : ::crospi_boost::parameter::aux::satisfies<
            ArgList
          , typename ::crospi_boost::parameter::aux
            ::as_parameter_requirements<ParameterSpec>::type
        >::type
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

