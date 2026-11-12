// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_CONFIG_050403_HPP
#define CROSPI_BOOST_PARAMETER_CONFIG_050403_HPP

#include <crospi_boost/config.hpp>
#include <crospi_boost/config/workaround.hpp>

// Allow projects to #define CROSPI_BOOST_PARAMETER_DISABLE_PERFECT_FORWARDING to
// turn off perfect forwarding as necessary.  Otherwise, also require correct
// SFINAE support, needed explicitly by tagged_argument & keyword & cast;
// correct function template ordering, needed by the code generation macros;
// rvalue references, needed throughout; variadic templates, needed by
// parameters; function template default arguments, needed by the code
// generation macros; and the ability to handle multiple parameter packs,
// needed by parameters.  Older versions of GCC either don't have the latter
// ability or cannot disambiguate between keyword's overloaded
// operators.  Older versions of Clang either fail to compile due to
// differences in length between parameter packs 'Args' and 'args' or fail at
// runtime due to segmentation faults.
// -- Cromwell D. Enage
#if !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING) && \
    !defined(CROSPI_BOOST_PARAMETER_DISABLE_PERFECT_FORWARDING) && \
    !defined(CROSPI_BOOST_NO_SFINAE) && \
    !defined(CROSPI_BOOST_NO_FUNCTION_TEMPLATE_ORDERING) && \
    !defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES) && \
    !defined(CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES) && \
    !defined(CROSPI_BOOST_NO_CXX11_FUNCTION_TEMPLATE_DEFAULT_ARGS) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564)) && !( \
        defined(CROSPI_BOOST_CLANG) && (1 == CROSPI_BOOST_CLANG) && ( \
            (__clang_major__ < 3) || ( \
                (3 == __clang_major__) && (__clang_minor__ < 2) \
            ) \
        ) \
    ) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 40900)
#define CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING
#endif

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) && \
    !defined(CROSPI_BOOST_PARAMETER_DISABLE_MP11_USAGE) && \
    !defined(CROSPI_BOOST_NO_CXX11_CONSTEXPR) && \
    !defined(CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276) && \
    !defined(CROSPI_BOOST_NO_CXX11_AUTO_DECLARATIONS) && \
    !defined(CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES) && \
    !defined(CROSPI_BOOST_NO_CXX11_STATIC_ASSERT) && \
    !defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS) && \
    !defined(CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST) && \
    !defined(CROSPI_BOOST_NO_CXX11_HDR_TUPLE)
// Boost.MP11 requires C++11. -- Cromwell D. Enage
#define CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif
#if !defined(CROSPI_BOOST_PARAMETER_MAX_ARITY)
// Unlike the variadic MPL sequences provided by Boost.Fusion,
// crospi_boost::mpl::vector has a size limit. -- Cromwell D. Enage
#include <crospi_boost/mpl/limits/vector.hpp>
#define CROSPI_BOOST_PARAMETER_MAX_ARITY CROSPI_BOOST_MPL_LIMIT_VECTOR_SIZE
#endif
#else   // !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#if !defined(CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY)
#define CROSPI_BOOST_PARAMETER_EXPONENTIAL_OVERLOAD_THRESHOLD_ARITY 0
#endif
#if !defined(CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY)
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1800) || \
    CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 60000)
// Some tests cause MSVC-11.0 and earlier to run out of heap space,
// while some other tests cause GCC 5 and earlier to do the same,
// if the value is set any higher. -- Cromwell D. Enage
#define CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY 20
#else
#define CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY 64
#endif
#endif  // CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY
#if !defined(CROSPI_BOOST_PARAMETER_MAX_ARITY)
#define CROSPI_BOOST_PARAMETER_MAX_ARITY 8
#endif
#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING
#endif  // include guard

