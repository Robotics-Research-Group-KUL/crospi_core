// Copyright David Abrahams 2005.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_RESULT_OF0_DWA2005511_HPP
#define CROSPI_BOOST_PARAMETER_AUX_RESULT_OF0_DWA2005511_HPP

#include <crospi_boost/parameter/aux_/use_default_tag.hpp>
#include <crospi_boost/parameter/config.hpp>
#include <crospi_boost/utility/result_of.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/type_traits/is_void.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // A metafunction returning the result of invoking
    // a nullary function object of the given type.
    template <typename F>
    class result_of0
    {
#if defined(CROSPI_BOOST_NO_RESULT_OF)
        typedef typename F::result_type result_of_F;
#else
        typedef typename ::crospi_boost::result_of<F()>::type result_of_F;
#endif

     public:
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = ::crospi_boost::mp11::mp_if<
            ::std::is_void<result_of_F>
#else
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_void<result_of_F>
#endif
          , ::crospi_boost::parameter::aux::use_default_tag
          , result_of_F
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        >;
#else
        >::type type;
#endif
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

