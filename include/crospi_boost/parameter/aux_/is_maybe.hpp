// Copyright Daniel Wallin, David Abrahams 2010.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_IS_MAYBE_050329_HPP
#define CROSPI_BOOST_PARAMETER_IS_MAYBE_050329_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    struct maybe_base
    {
    };
}}} // namespace crospi_boost::parameter::aux

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    using is_maybe = ::std::is_base_of<
        ::crospi_boost::parameter::aux::maybe_base
      , typename ::std::remove_const<T>::type
    >;
}}} // namespace crospi_boost::parameter::aux

#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/type_traits/is_base_of.hpp>
#include <crospi_boost/type_traits/remove_const.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_maybe
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::is_base_of<
                ::crospi_boost::parameter::aux::maybe_base
              , typename ::crospi_boost::remove_const<T>::type
            >
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::false_
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

