// Copyright Cromwell D. Enage 2019.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_ALWAYS_TRUE_PREDICATE_HPP
#define CROSPI_BOOST_PARAMETER_AUX_ALWAYS_TRUE_PREDICATE_HPP

#include <crospi_boost/parameter/config.hpp>
#include <crospi_boost/mpl/bool.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#endif
#else
#include <crospi_boost/mpl/always.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    struct always_true_predicate
    {
        template <typename ...>
        struct apply
        {
            typedef ::crospi_boost::mpl::true_ type;
        };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        template <typename ...>
        using fn = ::crospi_boost::mp11::mp_true;
#endif
    };
#else
    typedef ::crospi_boost::mpl::always< ::crospi_boost::mpl::true_> always_true_predicate;
#endif  // CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

