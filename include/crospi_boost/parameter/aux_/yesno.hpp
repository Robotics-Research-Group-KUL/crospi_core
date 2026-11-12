// Copyright Daniel Wallin, David Abrahams 2005.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_YESNO_HPP
#define CROSPI_BOOST_PARAMETER_AUX_YESNO_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    // types used with the "sizeof trick" to capture the results of
    // overload resolution at compile-time.
    typedef char yes_tag;
    typedef char (&no_tag)[2];
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/bool.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // mpl::true_ and mpl::false_ are not distinguishable by sizeof(),
    // so we pass them through these functions to get a type that is.
    ::crospi_boost::parameter::aux::yes_tag to_yesno(::crospi_boost::mpl::true_);
    ::crospi_boost::parameter::aux::no_tag to_yesno(::crospi_boost::mpl::false_);
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // mp11::mp_true and mp11::mp_false are not distinguishable by sizeof(),
    // so we pass them through these functions to get a type that is.
    ::crospi_boost::parameter::aux::yes_tag to_yesno(::crospi_boost::mp11::mp_true);
    ::crospi_boost::parameter::aux::no_tag to_yesno(::crospi_boost::mp11::mp_false);
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

