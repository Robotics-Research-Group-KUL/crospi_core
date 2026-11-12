// Copyright Daniel Wallin 2006.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_TEMPLATE_KEYWORD_060203_HPP
#define CROSPI_BOOST_PARAMETER_TEMPLATE_KEYWORD_060203_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    struct template_keyword_base
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    using is_template_keyword = ::std::is_base_of<
        ::crospi_boost::parameter::aux::template_keyword_base
      , typename ::std::remove_const<
            typename ::std::remove_reference<T>::type
        >::type
    >;
}}} // namespace crospi_boost::parameter::aux

#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/type_traits/remove_const.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#include <crospi_boost/type_traits/is_base_of.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>
#else
#include <crospi_boost/type_traits/is_convertible.hpp>
#include <crospi_boost/type_traits/is_lvalue_reference.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

#if !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    template <typename T>
    struct is_template_keyword_aux
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::is_convertible<
                T*
              , ::crospi_boost::parameter::aux::template_keyword_base const*
            >
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::false_
        >::type
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING

    template <typename T>
    struct is_template_keyword
      : ::crospi_boost::mpl::if_<
#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
            // Cannot use is_convertible<> to check if T is derived from
            // template_keyword_base. -- Cromwell D. Enage
            ::crospi_boost::is_base_of<
                ::crospi_boost::parameter::aux::template_keyword_base
              , typename ::crospi_boost::remove_const<
                    typename ::crospi_boost::remove_reference<T>::type
                >::type
            >
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::false_
#else
            ::crospi_boost::is_lvalue_reference<T>
          , ::crospi_boost::mpl::false_
          , ::crospi_boost::parameter::aux::is_template_keyword_aux<T>
#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

