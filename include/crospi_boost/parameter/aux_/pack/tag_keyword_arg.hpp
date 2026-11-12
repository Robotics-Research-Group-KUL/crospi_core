// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_KEYWORD_ARG_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_KEYWORD_ARG_HPP

#include <crospi_boost/parameter/aux_/tag.hpp>
#include <crospi_boost/parameter/config.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    struct tag_keyword_arg
    {
        template <typename K, typename T>
        struct apply
        {
            typedef typename ::crospi_boost::parameter::aux::tag<K,T>::type type;
        };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        template <typename K, typename T>
        using fn = typename ::crospi_boost::parameter::aux::tag<K,T>::type;
#endif
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

