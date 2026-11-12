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
# ifndef CROSPI_BOOST_PREPROCESSOR_REPETITION_ENUM_PARAMS_HPP
# define CROSPI_BOOST_PREPROCESSOR_REPETITION_ENUM_PARAMS_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/punctuation/comma_if.hpp>
# include <crospi_boost/preprocessor/repetition/repeat.hpp>
#
# /* CROSPI_BOOST_PP_ENUM_PARAMS */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ENUM_PARAMS(count, param) CROSPI_BOOST_PP_REPEAT(count, CROSPI_BOOST_PP_ENUM_PARAMS_M, param)
# else
#    define CROSPI_BOOST_PP_ENUM_PARAMS(count, param) CROSPI_BOOST_PP_ENUM_PARAMS_I(count, param)
#    define CROSPI_BOOST_PP_ENUM_PARAMS_I(count, param) CROSPI_BOOST_PP_REPEAT(count, CROSPI_BOOST_PP_ENUM_PARAMS_M, param)
# endif
#
# define CROSPI_BOOST_PP_ENUM_PARAMS_M(z, n, param) CROSPI_BOOST_PP_COMMA_IF(n) param ## n
#
# /* CROSPI_BOOST_PP_ENUM_PARAMS_Z */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ENUM_PARAMS_Z(z, count, param) CROSPI_BOOST_PP_REPEAT_ ## z(count, CROSPI_BOOST_PP_ENUM_PARAMS_M, param)
# else
#    define CROSPI_BOOST_PP_ENUM_PARAMS_Z(z, count, param) CROSPI_BOOST_PP_ENUM_PARAMS_Z_I(z, count, param)
#    define CROSPI_BOOST_PP_ENUM_PARAMS_Z_I(z, count, param) CROSPI_BOOST_PP_REPEAT_ ## z(count, CROSPI_BOOST_PP_ENUM_PARAMS_M, param)
# endif
#
# endif
