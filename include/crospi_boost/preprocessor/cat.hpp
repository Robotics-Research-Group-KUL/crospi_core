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
# ifndef CROSPI_BOOST_PREPROCESSOR_CAT_HPP
# define CROSPI_BOOST_PREPROCESSOR_CAT_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_CAT */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_CAT(a, b) CROSPI_BOOST_PP_CAT_I(a, b)
# else
#    define CROSPI_BOOST_PP_CAT(a, b) CROSPI_BOOST_PP_CAT_OO((a, b))
#    define CROSPI_BOOST_PP_CAT_OO(par) CROSPI_BOOST_PP_CAT_I ## par
# endif
#
# if (~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()) || (defined(__INTEL_COMPILER) && __INTEL_COMPILER >= 1700)
#    define CROSPI_BOOST_PP_CAT_I(a, b) a ## b
# else
#    define CROSPI_BOOST_PP_CAT_I(a, b) CROSPI_BOOST_PP_CAT_II(~, a ## b)
#    define CROSPI_BOOST_PP_CAT_II(p, res) res
# endif
#
# endif
