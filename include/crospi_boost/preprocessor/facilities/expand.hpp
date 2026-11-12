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
# ifndef CROSPI_BOOST_PREPROCESSOR_FACILITIES_EXPAND_HPP
# define CROSPI_BOOST_PREPROCESSOR_FACILITIES_EXPAND_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC() && ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_DMC()
#    define CROSPI_BOOST_PP_EXPAND(x) CROSPI_BOOST_PP_EXPAND_I(x)
# else
#    define CROSPI_BOOST_PP_EXPAND(x) CROSPI_BOOST_PP_EXPAND_OO((x))
#    define CROSPI_BOOST_PP_EXPAND_OO(par) CROSPI_BOOST_PP_EXPAND_I ## par
# endif
#
# define CROSPI_BOOST_PP_EXPAND_I(x) x
#
# endif
