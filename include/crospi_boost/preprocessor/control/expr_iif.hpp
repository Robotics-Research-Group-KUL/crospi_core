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
# ifndef CROSPI_BOOST_PREPROCESSOR_CONTROL_EXPR_IIF_HPP
# define CROSPI_BOOST_PREPROCESSOR_CONTROL_EXPR_IIF_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_EXPR_IIF */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_EXPR_IIF(bit, expr) CROSPI_BOOST_PP_EXPR_IIF_I(bit, expr)
# else
#    define CROSPI_BOOST_PP_EXPR_IIF(bit, expr) CROSPI_BOOST_PP_EXPR_IIF_OO((bit, expr))
#    define CROSPI_BOOST_PP_EXPR_IIF_OO(par) CROSPI_BOOST_PP_EXPR_IIF_I ## par
# endif
#
# define CROSPI_BOOST_PP_EXPR_IIF_I(bit, expr) CROSPI_BOOST_PP_EXPR_IIF_ ## bit(expr)
#
# define CROSPI_BOOST_PP_EXPR_IIF_0(expr)
# define CROSPI_BOOST_PP_EXPR_IIF_1(expr) expr
#
# endif
