# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2020.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_ARITHMETIC_DETAIL_IS_MAXIMUM_NUMBER_HPP
# define CROSPI_BOOST_PREPROCESSOR_ARITHMETIC_DETAIL_IS_MAXIMUM_NUMBER_HPP
#
# /* CROSPI_BOOST_PP_DETAIL_IS_MAXIMUM_NUMBER */
#
# include <crospi_boost/preprocessor/comparison/equal.hpp>
# include <crospi_boost/preprocessor/arithmetic/detail/maximum_number.hpp>
#
# define CROSPI_BOOST_PP_DETAIL_IS_MAXIMUM_NUMBER(x) CROSPI_BOOST_PP_EQUAL(x,CROSPI_BOOST_PP_DETAIL_MAXIMUM_NUMBER)
#
# endif
