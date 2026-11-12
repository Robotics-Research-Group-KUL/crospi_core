// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_REQUIRED_HPP
#define CROSPI_BOOST_PARAMETER_REQUIRED_HPP

#include <crospi_boost/parameter/aux_/use_default.hpp>

namespace crospi_boost { namespace parameter {

    // This metafunction can be used to describe the treatment of particular
    // named parameters for the purposes of overload elimination with SFINAE,
    // by placing specializations in the parameters<...> list.  In order for
    // a treated function to participate in overload resolution:
    //
    //   - all keyword tags wrapped in required<...> must have a matching
    //     actual argument
    //
    //   - The actual argument type matched by every keyword tag
    //     associated with a predicate must satisfy that predicate
    template <
        typename Tag
      , typename Predicate = ::crospi_boost::parameter::aux::use_default
    >
    struct required
    {
        typedef Tag key_type;
        typedef Predicate predicate;
    };
}}

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_required
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_false
#else
      : ::crospi_boost::mpl::false_
#endif
    {
    };

    template <typename Tag, typename Predicate>
    struct is_required< ::crospi_boost::parameter::required<Tag,Predicate> >
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_true
#else
      : ::crospi_boost::mpl::true_
#endif
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

