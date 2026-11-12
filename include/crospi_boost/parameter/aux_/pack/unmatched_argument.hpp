// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_UNMATCHED_ARGUMENT_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_UNMATCHED_ARGUMENT_HPP

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/assert.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct unmatched_argument
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        static_assert(::std::is_same<T,void>::value, "T == void");
#else
        CROSPI_BOOST_MPL_ASSERT((
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::is_same<T,void>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::mpl::false_
            >::type
        ));
#endif
        typedef int type;
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

