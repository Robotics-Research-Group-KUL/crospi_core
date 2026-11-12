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
# ifndef CROSPI_BOOST_PREPROCESSOR_LOGICAL_BITOR_HPP
# define CROSPI_BOOST_PREPROCESSOR_LOGICAL_BITOR_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_BITOR */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_BITOR(x, y) CROSPI_BOOST_PP_BITOR_I(x, y)
# else
#    define CROSPI_BOOST_PP_BITOR(x, y) CROSPI_BOOST_PP_BITOR_OO((x, y))
#    define CROSPI_BOOST_PP_BITOR_OO(par) CROSPI_BOOST_PP_BITOR_I ## par
# endif
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()
#    define CROSPI_BOOST_PP_BITOR_I(x, y) CROSPI_BOOST_PP_BITOR_ ## x ## y
# else
#    define CROSPI_BOOST_PP_BITOR_I(x, y) CROSPI_BOOST_PP_BITOR_ID(CROSPI_BOOST_PP_BITOR_ ## x ## y)
#    define CROSPI_BOOST_PP_BITOR_ID(id) id
# endif
#
# define CROSPI_BOOST_PP_BITOR_00 0
# define CROSPI_BOOST_PP_BITOR_01 1
# define CROSPI_BOOST_PP_BITOR_10 1
# define CROSPI_BOOST_PP_BITOR_11 1
#
# endif
