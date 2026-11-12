// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_IS_TAGGED_ARGUMENT_HPP
#define CROSPI_BOOST_PARAMETER_IS_TAGGED_ARGUMENT_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    struct tagged_argument_base
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING) || \
    (0 < CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY)
#include <crospi_boost/type_traits/is_base_of.hpp>
#include <crospi_boost/type_traits/remove_const.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // This metafunction identifies tagged_argument specializations
    // and their derived classes.
    template <typename T>
    struct is_tagged_argument
      : ::crospi_boost::mpl::if_<
            // Cannot use is_convertible<> to check if T is derived from
            // tagged_argument_base. -- Cromwell D. Enage
            ::crospi_boost::is_base_of<
                ::crospi_boost::parameter::aux::tagged_argument_base
              , typename ::crospi_boost::remove_const<
                    typename ::crospi_boost::remove_reference<T>::type
                >::type
            >
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::false_
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#else   // no perfect forwarding support and no exponential overloads
#include <crospi_boost/type_traits/is_convertible.hpp>
#include <crospi_boost/type_traits/is_lvalue_reference.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_tagged_argument_aux
      : ::crospi_boost::is_convertible<
            T*
          , ::crospi_boost::parameter::aux::tagged_argument_base const*
        >
    {
    };

    // This metafunction identifies tagged_argument specializations
    // and their derived classes.
    template <typename T>
    struct is_tagged_argument
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::is_lvalue_reference<T>
          , ::crospi_boost::mpl::false_
          , ::crospi_boost::parameter::aux::is_tagged_argument_aux<T>
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // perfect forwarding support, or exponential overloads

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    using is_tagged_argument_mp11 = ::std::is_base_of<
        ::crospi_boost::parameter::aux::tagged_argument_base
      , typename ::std::remove_const<
            typename ::std::remove_reference<T>::type
        >::type
    >;
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

