// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_IS_NAMED_ARGUMENT_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_IS_NAMED_ARGUMENT_HPP

#include <crospi_boost/parameter/aux_/template_keyword.hpp>
#include <crospi_boost/parameter/aux_/is_tagged_argument.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using is_named_argument = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::parameter::aux::is_template_keyword<T>
      , ::crospi_boost::mp11::mp_true
      , ::crospi_boost::parameter::aux::is_tagged_argument_mp11<T>
    >;
#else
    struct is_named_argument
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::parameter::aux::is_template_keyword<T>
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::parameter::aux::is_tagged_argument<T>
        >::type
    {
    };
#endif
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

