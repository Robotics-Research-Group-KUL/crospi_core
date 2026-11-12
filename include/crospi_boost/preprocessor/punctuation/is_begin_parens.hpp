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
# ifndef CROSPI_BOOST_PREPROCESSOR_IS_BEGIN_PARENS_HPP
# define CROSPI_BOOST_PREPROCESSOR_IS_BEGIN_PARENS_HPP

#include <crospi_boost/preprocessor/config/config.hpp>
#include <crospi_boost/preprocessor/punctuation/detail/is_begin_parens.hpp>

#if CROSPI_BOOST_PP_VARIADICS_MSVC && _MSC_VER <= 1400

#define CROSPI_BOOST_PP_IS_BEGIN_PARENS(param) \
    CROSPI_BOOST_PP_DETAIL_IBP_SPLIT \
      ( \
      0, \
      CROSPI_BOOST_PP_DETAIL_IBP_CAT \
        ( \
        CROSPI_BOOST_PP_DETAIL_IBP_IS_VARIADIC_R_, \
        CROSPI_BOOST_PP_DETAIL_IBP_IS_VARIADIC_C param \
        ) \
      ) \
/**/

#else

#define CROSPI_BOOST_PP_IS_BEGIN_PARENS(...) \
    CROSPI_BOOST_PP_DETAIL_IBP_SPLIT \
      ( \
      0, \
      CROSPI_BOOST_PP_DETAIL_IBP_CAT \
        ( \
        CROSPI_BOOST_PP_DETAIL_IBP_IS_VARIADIC_R_, \
        CROSPI_BOOST_PP_DETAIL_IBP_IS_VARIADIC_C __VA_ARGS__ \
        ) \
      ) \
/**/

#endif /* CROSPI_BOOST_PP_VARIADICS_MSVC && _MSC_VER <= 1400 */
#endif /* CROSPI_BOOST_PREPROCESSOR_IS_BEGIN_PARENS_HPP */
