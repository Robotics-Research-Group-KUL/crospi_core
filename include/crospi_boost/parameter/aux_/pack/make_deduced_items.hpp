// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_MAKE_DEDUCED_ITEMS_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_MAKE_DEDUCED_ITEMS_HPP

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/aux_/pack/deduced_item.hpp>
#include <crospi_boost/parameter/deduced.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Spec, typename Tail>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using make_deduced_items = ::crospi_boost::mp11::mp_if<
        ::std::is_same<Spec,::crospi_boost::parameter::void_>
      , ::crospi_boost::mp11::mp_identity< ::crospi_boost::parameter::void_>
      , ::crospi_boost::mp11::mp_if<
            ::crospi_boost::parameter::aux::is_deduced<Spec>
          , ::crospi_boost::parameter::aux::make_deduced_item<Spec,Tail>
          , Tail
        >
    >;
#else
    struct make_deduced_items
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<Spec,::crospi_boost::parameter::void_>
          , ::crospi_boost::mpl::identity< ::crospi_boost::parameter::void_>
          , ::crospi_boost::mpl::eval_if<
                ::crospi_boost::parameter::aux::is_deduced<Spec>
              , ::crospi_boost::parameter::aux::make_deduced_item<Spec,Tail>
              , Tail
            >
        >
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

