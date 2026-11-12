# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_COMPARISON_EQUAL_HPP
# define CROSPI_BOOST_PREPROCESSOR_COMPARISON_EQUAL_HPP
#
# include <crospi_boost/preprocessor/comparison/not_equal.hpp>
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/logical/compl.hpp>
#
# /* CROSPI_BOOST_PP_EQUAL */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_EQUAL(x, y) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_NOT_EQUAL(x, y))
# else
#    define CROSPI_BOOST_PP_EQUAL(x, y) CROSPI_BOOST_PP_EQUAL_I(x, y)
#    define CROSPI_BOOST_PP_EQUAL_I(x, y) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_NOT_EQUAL(x, y))
# endif
#
# /* CROSPI_BOOST_PP_EQUAL_D */
#
# define CROSPI_BOOST_PP_EQUAL_D(d, x, y) CROSPI_BOOST_PP_EQUAL(x, y)
#
# endif
