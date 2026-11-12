# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
#ifndef CROSPI_BOOST_PREPROCESSOR_DETAIL_IS_EMPTY_HPP
#define CROSPI_BOOST_PREPROCESSOR_DETAIL_IS_EMPTY_HPP

#include <crospi_boost/preprocessor/punctuation/is_begin_parens.hpp>

#if CROSPI_BOOST_PP_VARIADICS_MSVC

# pragma warning(once:4002)

#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF_0(t, b) b
#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF_1(t, b) t

#else

#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF_0(t, ...) __VA_ARGS__
#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF_1(t, ...) t

#endif

#if CROSPI_BOOST_PP_VARIADICS_MSVC && _MSC_VER <= 1400

#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PROCESS(param) \
    CROSPI_BOOST_PP_IS_BEGIN_PARENS \
        ( \
        CROSPI_BOOST_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C param () \
        ) \
/**/

#else

#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PROCESS(...) \
    CROSPI_BOOST_PP_IS_BEGIN_PARENS \
        ( \
        CROSPI_BOOST_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C __VA_ARGS__ () \
        ) \
/**/

#endif

#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PRIMITIVE_CAT(a, b) a ## b
#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF(bit) CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PRIMITIVE_CAT(CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF_,bit)
#define CROSPI_BOOST_PP_DETAIL_IS_EMPTY_NON_FUNCTION_C(...) ()

#endif /* CROSPI_BOOST_PREPROCESSOR_DETAIL_IS_EMPTY_HPP */
