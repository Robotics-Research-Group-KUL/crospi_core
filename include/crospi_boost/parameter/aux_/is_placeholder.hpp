// Copyright Cromwell D. Enage 2019.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_IS_PLACEHOLDER_HPP
#define CROSPI_BOOST_PARAMETER_AUX_IS_PLACEHOLDER_HPP

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_mpl_placeholder
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_false
#else
      : ::crospi_boost::mpl::false_
#endif
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/arg_fwd.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <int I>
    struct is_mpl_placeholder< ::crospi_boost::mpl::arg<I> >
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_true
#else
      : ::crospi_boost::mpl::true_
#endif
    {
    };
}}} // namespace crospi_boost::parameter::aux

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/bind.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_mp11_placeholder : ::crospi_boost::mp11::mp_false
    {
    };

    template < ::std::size_t I>
    struct is_mp11_placeholder< ::crospi_boost::mp11::mp_arg<I> >
      : ::crospi_boost::mp11::mp_true
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

