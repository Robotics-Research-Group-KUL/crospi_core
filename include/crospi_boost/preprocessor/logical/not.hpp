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
# ifndef CROSPI_BOOST_PREPROCESSOR_LOGICAL_NOT_HPP
# define CROSPI_BOOST_PREPROCESSOR_LOGICAL_NOT_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/logical/bool.hpp>
# include <crospi_boost/preprocessor/logical/compl.hpp>
#
# /* CROSPI_BOOST_PP_NOT */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_NOT(x) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_BOOL(x))
# else
#    define CROSPI_BOOST_PP_NOT(x) CROSPI_BOOST_PP_NOT_I(x)
#    define CROSPI_BOOST_PP_NOT_I(x) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_BOOL(x))
# endif
#
# endif
