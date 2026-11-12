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
# ifndef CROSPI_BOOST_PREPROCESSOR_STRINGIZE_HPP
# define CROSPI_BOOST_PREPROCESSOR_STRINGIZE_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# /* CROSPI_BOOST_PP_STRINGIZE */
#
# if CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MSVC()
#    define CROSPI_BOOST_PP_STRINGIZE(text) CROSPI_BOOST_PP_STRINGIZE_A((text))
#    define CROSPI_BOOST_PP_STRINGIZE_A(arg) CROSPI_BOOST_PP_STRINGIZE_I arg
# elif CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_STRINGIZE(text) CROSPI_BOOST_PP_STRINGIZE_OO((text))
#    define CROSPI_BOOST_PP_STRINGIZE_OO(par) CROSPI_BOOST_PP_STRINGIZE_I ## par
# else
#    define CROSPI_BOOST_PP_STRINGIZE(text) CROSPI_BOOST_PP_STRINGIZE_I(text)
# endif
#
# define CROSPI_BOOST_PP_STRINGIZE_I(...) #__VA_ARGS__
#
# endif
