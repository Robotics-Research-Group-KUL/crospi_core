// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_DEFAULT_HPP
#define CROSPI_BOOST_PARAMETER_AUX_DEFAULT_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    // A wrapper for the default value passed by the user when resolving
    // the value of the parameter with the given Keyword
    template <typename Keyword, typename Value>
    struct default_
    {
        inline CROSPI_BOOST_CONSTEXPR default_(Value& x) : value(x)
        {
        }

        Value& value;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // lazy_default -- A wrapper for the default value computation function
    // passed by the user when resolving the value of the parameter with the
    // given keyword.
#if CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 300)
    // These compilers need a little extra help with overload resolution;
    // we have empty_arg_list's operator[] accept a base class
    // to make that overload less preferable.
    template <typename KW, typename DefaultComputer>
    struct lazy_default_base
    {
        inline CROSPI_BOOST_CONSTEXPR lazy_default_base(DefaultComputer& x)
          : compute_default(x)
        {
        }

        DefaultComputer& compute_default;
    };

    template <typename KW, typename DefaultComputer>
    struct lazy_default
      : ::crospi_boost::parameter::aux::lazy_default_base<KW,DefaultComputer>
    {
        inline CROSPI_BOOST_CONSTEXPR lazy_default(DefaultComputer& x)
          : ::crospi_boost::parameter::aux::lazy_default_base<KW,DefaultComputer>(x)
        {
        }
    };
#else   // !CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 300)
    template <typename KW, typename DefaultComputer>
    struct lazy_default
    {
        inline CROSPI_BOOST_CONSTEXPR lazy_default(DefaultComputer& x)
          : compute_default(x)
        {
        }

        DefaultComputer& compute_default;
    };
#endif  // EDG workarounds needed.
}}} // namespace crospi_boost::parameter::aux

#if CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, <= 300)
#define CROSPI_BOOST_PARAMETER_lazy_default_fallback \
    ::crospi_boost::parameter::aux::lazy_default_base
/**/
#else
#define CROSPI_BOOST_PARAMETER_lazy_default_fallback \
    ::crospi_boost::parameter::aux::lazy_default
/**/
#endif

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

#include <utility>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Value>
    struct default_r_
    {
        inline CROSPI_BOOST_CONSTEXPR default_r_(Value&& x)
          : value(::std::forward<Value>(x))
        {
        }

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1910)
        // MSVC 2015 miscompiles moves for classes containing rvalue ref members
        // using the default generated move constructor
        // when moving into a function
        // https://github.com/boostorg/parameter/pull/109
        inline CROSPI_BOOST_CONSTEXPR default_r_(default_r_&& x)
          : value(::std::forward<Value>(x.value))
        {
        }
#endif

        Value&& value;
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING
#endif  // include guard

