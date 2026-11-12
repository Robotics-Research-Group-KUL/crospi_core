// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_PREDICATE_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_PREDICATE_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    // helper for get_predicate<...>, below
    template <typename T>
    struct get_predicate_or_default
    {
        typedef T type;
    };

    // helper for predicate<...>, below
    template <typename T>
    struct get_predicate
      : ::crospi_boost::parameter::aux
        ::get_predicate_or_default<typename T::predicate>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/use_default.hpp>
#include <crospi_boost/parameter/aux_/always_true_predicate.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <>
    struct get_predicate_or_default< ::crospi_boost::parameter::aux::use_default>
    {
        typedef ::crospi_boost::parameter::aux::always_true_predicate type;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/required.hpp>
#include <crospi_boost/parameter/optional.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using predicate = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_if<
            ::crospi_boost::parameter::aux::is_optional<T>
          , ::crospi_boost::mp11::mp_true
          , ::crospi_boost::parameter::aux::is_required<T>
        >
      , ::crospi_boost::parameter::aux::get_predicate<T>
      , ::crospi_boost::mp11::mp_identity<
            ::crospi_boost::parameter::aux::always_true_predicate
        >
    >;
#else
    struct predicate
      : ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::parameter::aux::is_optional<T>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::parameter::aux::is_required<T>
            >::type
          , ::crospi_boost::parameter::aux::get_predicate<T>
          , ::crospi_boost::mpl::identity<
                ::crospi_boost::parameter::aux::always_true_predicate
            >
        >
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

