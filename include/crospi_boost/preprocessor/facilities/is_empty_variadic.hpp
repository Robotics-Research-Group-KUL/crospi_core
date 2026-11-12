# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Edward Diener 2014,2019.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP
# define CROSPI_BOOST_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/punctuation/is_begin_parens.hpp>
# include <crospi_boost/preprocessor/facilities/detail/is_empty.hpp>
#
#if CROSPI_BOOST_PP_VARIADICS_MSVC && _MSC_VER <= 1400
#
#define CROSPI_BOOST_PP_IS_EMPTY(param) \
    CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF \
      ( \
      CROSPI_BOOST_PP_IS_BEGIN_PARENS \
        ( \
        param \
        ) \
      ) \
      ( \
      CROSPI_BOOST_PP_IS_EMPTY_ZERO, \
      CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PROCESS \
      ) \
    (param) \
/**/
#define CROSPI_BOOST_PP_IS_EMPTY_ZERO(param) 0
# else
# if defined(__cplusplus) && __cplusplus > 201703L
# include <crospi_boost/preprocessor/variadic/has_opt.hpp>
#define CROSPI_BOOST_PP_IS_EMPTY(...) \
    CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF \
      ( \
      CROSPI_BOOST_PP_VARIADIC_HAS_OPT() \
      ) \
      ( \
      CROSPI_BOOST_PP_IS_EMPTY_OPT, \
      CROSPI_BOOST_PP_IS_EMPTY_NO_OPT \
      ) \
    (__VA_ARGS__) \
/**/
#define CROSPI_BOOST_PP_IS_EMPTY_FUNCTION2(...) \
    __VA_OPT__(0,) 1 \
/**/
#define CROSPI_BOOST_PP_IS_EMPTY_FUNCTION(...) \
    CROSPI_BOOST_PP_IS_EMPTY_FUNCTION2(__VA_ARGS__) \
/**/
#define CROSPI_BOOST_PP_IS_EMPTY_OPT(...) \
    CROSPI_BOOST_PP_VARIADIC_HAS_OPT_ELEM0(CROSPI_BOOST_PP_IS_EMPTY_FUNCTION(__VA_ARGS__),) \
/**/
# else
#define CROSPI_BOOST_PP_IS_EMPTY(...) \
    CROSPI_BOOST_PP_IS_EMPTY_NO_OPT(__VA_ARGS__) \
/**/
# endif /* defined(__cplusplus) && __cplusplus > 201703L */
#define CROSPI_BOOST_PP_IS_EMPTY_NO_OPT(...) \
    CROSPI_BOOST_PP_DETAIL_IS_EMPTY_IIF \
      ( \
      CROSPI_BOOST_PP_IS_BEGIN_PARENS \
        ( \
        __VA_ARGS__ \
        ) \
      ) \
      ( \
      CROSPI_BOOST_PP_IS_EMPTY_ZERO, \
      CROSPI_BOOST_PP_DETAIL_IS_EMPTY_PROCESS \
      ) \
    (__VA_ARGS__) \
/**/
#define CROSPI_BOOST_PP_IS_EMPTY_ZERO(...) 0
# endif /* CROSPI_BOOST_PP_VARIADICS_MSVC && _MSC_VER <= 1400 */
# endif /* CROSPI_BOOST_PREPROCESSOR_FACILITIES_IS_EMPTY_VARIADIC_HPP */
