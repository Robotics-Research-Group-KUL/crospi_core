// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_TYPE_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_TYPE_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    // helper for tag_type<...>, below.
    template <typename T>
    struct get_tag_type0
    {
        typedef typename T::key_type type;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/deduced.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/eval_if.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct get_tag_type
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_if<
#else
      : ::crospi_boost::mpl::eval_if<
#endif
            ::crospi_boost::parameter::aux::is_deduced0<T>
          , ::crospi_boost::parameter::aux::get_tag_type0<typename T::key_type>
          , ::crospi_boost::parameter::aux::get_tag_type0<T>
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/required.hpp>
#include <crospi_boost/parameter/optional.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    using tag_type = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_if<
            ::crospi_boost::parameter::aux::is_optional<T>
          , ::crospi_boost::mp11::mp_true
          , ::crospi_boost::parameter::aux::is_required<T>
        >
      , ::crospi_boost::parameter::aux::get_tag_type<T>
      , ::crospi_boost::mp11::mp_identity<T>
    >;
}}} // namespace crospi_boost::parameter::aux

#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/identity.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct tag_type
      : ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::parameter::aux::is_optional<T>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::parameter::aux::is_required<T>
            >::type
          , ::crospi_boost::parameter::aux::get_tag_type<T>
          , ::crospi_boost::mpl::identity<T>
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

