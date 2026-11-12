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
# ifndef CROSPI_BOOST_PREPROCESSOR_LOGICAL_BITAND_HPP
# define CROSPI_BOOST_PREPROCESSOR_LOGICAL_BITAND_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_BITAND */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_BITAND(x, y) CROSPI_BOOST_PP_BITAND_I(x, y)
# else
#    define CROSPI_BOOST_PP_BITAND(x, y) CROSPI_BOOST_PP_BITAND_OO((x, y))
#    define CROSPI_BOOST_PP_BITAND_OO(par) CROSPI_BOOST_PP_BITAND_I ## par
# endif
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()
#    define CROSPI_BOOST_PP_BITAND_I(x, y) CROSPI_BOOST_PP_BITAND_ ## x ## y
# else
#    define CROSPI_BOOST_PP_BITAND_I(x, y) CROSPI_BOOST_PP_BITAND_ID(CROSPI_BOOST_PP_BITAND_ ## x ## y)
#    define CROSPI_BOOST_PP_BITAND_ID(res) res
# endif
#
# define CROSPI_BOOST_PP_BITAND_00 0
# define CROSPI_BOOST_PP_BITAND_01 0
# define CROSPI_BOOST_PP_BITAND_10 0
# define CROSPI_BOOST_PP_BITAND_11 1
#
# endif
