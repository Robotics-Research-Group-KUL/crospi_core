// Copyright Daniel Wallin 2006.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_USE_DEFAULT_TAG_HPP
#define CROSPI_BOOST_PARAMETER_USE_DEFAULT_TAG_HPP

#include <crospi_boost/config.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    struct use_default_tag
    {
        inline CROSPI_BOOST_CONSTEXPR CROSPI_BOOST_DEFAULTED_FUNCTION(use_default_tag(), {})

        inline CROSPI_BOOST_CONSTEXPR CROSPI_BOOST_DEFAULTED_FUNCTION(
            use_default_tag(use_default_tag const&), {}
        )

        inline CROSPI_BOOST_CONSTEXPR use_default_tag operator()() const
        {
            return *this;
        }
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

