# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_DETAIL_IS_BINARY_HPP
# define CROSPI_BOOST_PREPROCESSOR_DETAIL_IS_BINARY_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/detail/check.hpp>
#
# /* CROSPI_BOOST_PP_IS_BINARY */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_IS_BINARY(x) CROSPI_BOOST_PP_CHECK(x, CROSPI_BOOST_PP_IS_BINARY_CHECK)
# else
#    define CROSPI_BOOST_PP_IS_BINARY(x) CROSPI_BOOST_PP_IS_BINARY_I(x)
#    define CROSPI_BOOST_PP_IS_BINARY_I(x) CROSPI_BOOST_PP_CHECK(x, CROSPI_BOOST_PP_IS_BINARY_CHECK)
# endif
#
# define CROSPI_BOOST_PP_IS_BINARY_CHECK(a, b) 1
# define CROSPI_BOOST_PP_CHECK_RESULT_CROSPI_BOOST_PP_IS_BINARY_CHECK 0, CROSPI_BOOST_PP_NIL
#
# endif
