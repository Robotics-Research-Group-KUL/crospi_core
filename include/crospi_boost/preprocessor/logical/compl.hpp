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
# ifndef CROSPI_BOOST_PREPROCESSOR_LOGICAL_COMPL_HPP
# define CROSPI_BOOST_PREPROCESSOR_LOGICAL_COMPL_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_COMPL */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_COMPL(x) CROSPI_BOOST_PP_COMPL_I(x)
# else
#    define CROSPI_BOOST_PP_COMPL(x) CROSPI_BOOST_PP_COMPL_OO((x))
#    define CROSPI_BOOST_PP_COMPL_OO(par) CROSPI_BOOST_PP_COMPL_I ## par
# endif
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()
#    define CROSPI_BOOST_PP_COMPL_I(x) CROSPI_BOOST_PP_COMPL_ ## x
# else
#    define CROSPI_BOOST_PP_COMPL_I(x) CROSPI_BOOST_PP_COMPL_ID(CROSPI_BOOST_PP_COMPL_ ## x)
#    define CROSPI_BOOST_PP_COMPL_ID(id) id
# endif
#
# define CROSPI_BOOST_PP_COMPL_0 1
# define CROSPI_BOOST_PP_COMPL_1 0
#
# endif
