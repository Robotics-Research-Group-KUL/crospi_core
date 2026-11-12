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
# ifndef CROSPI_BOOST_PREPROCESSOR_DEBUG_ERROR_HPP
# define CROSPI_BOOST_PREPROCESSOR_DEBUG_ERROR_HPP
#
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_ERROR */
#
# if CROSPI_BOOST_PP_CONFIG_ERRORS
#    define CROSPI_BOOST_PP_ERROR(code) CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_ERROR_, code)
# endif
#
# define CROSPI_BOOST_PP_ERROR_0x0000 CROSPI_BOOST_PP_ERROR(0x0000, CROSPI_BOOST_PP_INDEX_OUT_OF_BOUNDS)
# define CROSPI_BOOST_PP_ERROR_0x0001 CROSPI_BOOST_PP_ERROR(0x0001, CROSPI_BOOST_PP_WHILE_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0002 CROSPI_BOOST_PP_ERROR(0x0002, CROSPI_BOOST_PP_FOR_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0003 CROSPI_BOOST_PP_ERROR(0x0003, CROSPI_BOOST_PP_REPEAT_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0004 CROSPI_BOOST_PP_ERROR(0x0004, CROSPI_BOOST_PP_LIST_FOLD_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0005 CROSPI_BOOST_PP_ERROR(0x0005, CROSPI_BOOST_PP_SEQ_FOLD_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0006 CROSPI_BOOST_PP_ERROR(0x0006, CROSPI_BOOST_PP_ARITHMETIC_OVERFLOW)
# define CROSPI_BOOST_PP_ERROR_0x0007 CROSPI_BOOST_PP_ERROR(0x0007, CROSPI_BOOST_PP_DIVISION_BY_ZERO)
#
# endif
