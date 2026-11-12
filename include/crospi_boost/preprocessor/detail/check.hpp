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
# ifndef CROSPI_BOOST_PREPROCESSOR_DETAIL_CHECK_HPP
# define CROSPI_BOOST_PREPROCESSOR_DETAIL_CHECK_HPP
#
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_CHECK */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_CHECK(x, type) CROSPI_BOOST_PP_CHECK_D(x, type)
# else
#    define CROSPI_BOOST_PP_CHECK(x, type) CROSPI_BOOST_PP_CHECK_OO((x, type))
#    define CROSPI_BOOST_PP_CHECK_OO(par) CROSPI_BOOST_PP_CHECK_D ## par
# endif
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC() && ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_DMC()
#    define CROSPI_BOOST_PP_CHECK_D(x, type) CROSPI_BOOST_PP_CHECK_1(CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_CHECK_RESULT_, type x))
#    define CROSPI_BOOST_PP_CHECK_1(chk) CROSPI_BOOST_PP_CHECK_2(chk)
#    define CROSPI_BOOST_PP_CHECK_2(res, _) res
# elif CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()
#    define CROSPI_BOOST_PP_CHECK_D(x, type) CROSPI_BOOST_PP_CHECK_1(type x)
#    define CROSPI_BOOST_PP_CHECK_1(chk) CROSPI_BOOST_PP_CHECK_2(chk)
#    define CROSPI_BOOST_PP_CHECK_2(chk) CROSPI_BOOST_PP_CHECK_3((CROSPI_BOOST_PP_CHECK_RESULT_ ## chk))
#    define CROSPI_BOOST_PP_CHECK_3(im) CROSPI_BOOST_PP_CHECK_5(CROSPI_BOOST_PP_CHECK_4 im)
#    define CROSPI_BOOST_PP_CHECK_4(res, _) res
#    define CROSPI_BOOST_PP_CHECK_5(res) res
# else /* DMC */
#    define CROSPI_BOOST_PP_CHECK_D(x, type) CROSPI_BOOST_PP_CHECK_OO((type x))
#    define CROSPI_BOOST_PP_CHECK_OO(par) CROSPI_BOOST_PP_CHECK_0 ## par
#    define CROSPI_BOOST_PP_CHECK_0(chk) CROSPI_BOOST_PP_CHECK_1(CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_CHECK_RESULT_, chk))
#    define CROSPI_BOOST_PP_CHECK_1(chk) CROSPI_BOOST_PP_CHECK_2(chk)
#    define CROSPI_BOOST_PP_CHECK_2(res, _) res
# endif
#
# define CROSPI_BOOST_PP_CHECK_RESULT_1 1, CROSPI_BOOST_PP_NIL
#
# endif
