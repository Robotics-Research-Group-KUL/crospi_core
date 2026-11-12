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
# ifndef CROSPI_BOOST_PREPROCESSOR_LOGICAL_AND_HPP
# define CROSPI_BOOST_PREPROCESSOR_LOGICAL_AND_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/logical/bool.hpp>
# include <crospi_boost/preprocessor/logical/bitand.hpp>
#
# /* CROSPI_BOOST_PP_AND */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_AND(p, q) CROSPI_BOOST_PP_BITAND(CROSPI_BOOST_PP_BOOL(p), CROSPI_BOOST_PP_BOOL(q))
# else
#    define CROSPI_BOOST_PP_AND(p, q) CROSPI_BOOST_PP_AND_I(p, q)
#    define CROSPI_BOOST_PP_AND_I(p, q) CROSPI_BOOST_PP_BITAND(CROSPI_BOOST_PP_BOOL(p), CROSPI_BOOST_PP_BOOL(q))
# endif
#
# endif
