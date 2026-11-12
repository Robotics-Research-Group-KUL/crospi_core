# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_TUPLE_DETAIL_IS_SINGLE_RETURN_HPP
# define CROSPI_BOOST_PREPROCESSOR_TUPLE_DETAIL_IS_SINGLE_RETURN_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_TUPLE_IS_SINGLE_RETURN */
#
# if CROSPI_BOOST_PP_VARIADICS_MSVC
# include <crospi_boost/preprocessor/control/iif.hpp>
# include <crospi_boost/preprocessor/facilities/is_1.hpp>
# include <crospi_boost/preprocessor/tuple/size.hpp>
# define CROSPI_BOOST_PP_TUPLE_IS_SINGLE_RETURN(sr,nsr,tuple)  \
    CROSPI_BOOST_PP_IIF(CROSPI_BOOST_PP_IS_1(CROSPI_BOOST_PP_TUPLE_SIZE(tuple)),sr,nsr) \
    /**/
# endif /* CROSPI_BOOST_PP_VARIADICS_MSVC */
#
# endif /* CROSPI_BOOST_PREPROCESSOR_TUPLE_DETAIL_IS_SINGLE_RETURN_HPP */
