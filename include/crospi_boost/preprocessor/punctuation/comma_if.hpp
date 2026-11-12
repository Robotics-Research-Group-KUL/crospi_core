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
# ifndef CROSPI_BOOST_PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
# define CROSPI_BOOST_PREPROCESSOR_PUNCTUATION_COMMA_IF_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/control/if.hpp>
# include <crospi_boost/preprocessor/facilities/empty.hpp>
# include <crospi_boost/preprocessor/punctuation/comma.hpp>
#
# /* CROSPI_BOOST_PP_COMMA_IF */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_COMMA_IF(cond) CROSPI_BOOST_PP_IF(cond, CROSPI_BOOST_PP_COMMA, CROSPI_BOOST_PP_EMPTY)()
# else
#    define CROSPI_BOOST_PP_COMMA_IF(cond) CROSPI_BOOST_PP_COMMA_IF_I(cond)
#    define CROSPI_BOOST_PP_COMMA_IF_I(cond) CROSPI_BOOST_PP_IF(cond, CROSPI_BOOST_PP_COMMA, CROSPI_BOOST_PP_EMPTY)()
# endif
#
# endif
