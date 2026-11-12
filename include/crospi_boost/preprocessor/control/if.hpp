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
# ifndef CROSPI_BOOST_PREPROCESSOR_CONTROL_IF_HPP
# define CROSPI_BOOST_PREPROCESSOR_CONTROL_IF_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/control/iif.hpp>
# include <crospi_boost/preprocessor/logical/bool.hpp>
#
# /* CROSPI_BOOST_PP_IF */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_IF(cond, t, f) CROSPI_BOOST_PP_IIF(CROSPI_BOOST_PP_BOOL(cond), t, f)
# else
#    define CROSPI_BOOST_PP_IF(cond, t, f) CROSPI_BOOST_PP_IF_I(cond, t, f)
#    define CROSPI_BOOST_PP_IF_I(cond, t, f) CROSPI_BOOST_PP_IIF(CROSPI_BOOST_PP_BOOL(cond), t, f)
# endif
#
# endif
