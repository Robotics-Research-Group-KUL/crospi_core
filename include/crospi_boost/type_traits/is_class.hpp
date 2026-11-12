//  (C) Copyright Dave Abrahams, Steve Cleary, Beman Dawes, Howard
//  Hinnant & John Maddock 2000-2003.
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.


#ifndef CROSPI_BOOST_TT_IS_CLASS_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_CLASS_HPP_INCLUDED

#include <crospi_boost/type_traits/detail/config.hpp>
#include <crospi_boost/type_traits/intrinsics.hpp>
#include <crospi_boost/type_traits/integral_constant.hpp>
#ifndef CROSPI_BOOST_IS_CLASS
#   include <crospi_boost/type_traits/is_union.hpp>

#ifdef CROSPI_BOOST_TT_HAS_CONFORMING_IS_CLASS_IMPLEMENTATION
#   include <crospi_boost/type_traits/detail/yes_no_type.hpp>
#else
#   include <crospi_boost/type_traits/is_scalar.hpp>
#   include <crospi_boost/type_traits/is_array.hpp>
#   include <crospi_boost/type_traits/is_reference.hpp>
#   include <crospi_boost/type_traits/is_void.hpp>
#   include <crospi_boost/type_traits/is_function.hpp>
#endif

#endif // CROSPI_BOOST_IS_CLASS

namespace crospi_boost {

namespace detail {

#ifndef CROSPI_BOOST_IS_CLASS
#ifdef CROSPI_BOOST_TT_HAS_CONFORMING_IS_CLASS_IMPLEMENTATION

// This is actually the conforming implementation which works with
// abstract classes.  However, enough compilers have trouble with
// it that most will use the one in
// crospi_boost/type_traits/object_traits.hpp. This implementation
// actually works with VC7.0, but other interactions seem to fail
// when we use it.

// is_class<> metafunction due to Paul Mensonides
// (leavings@attbi.com). For more details:
// http://groups.google.com/groups?hl=en&selm=000001c1cc83%24e154d5e0%247772e50c%40c161550a&rnum=1
#if defined(__GNUC__)  && !defined(__EDG_VERSION__)

template <class U> ::crospi_boost::type_traits::yes_type is_class_tester(void(U::*)(void));
template <class U> ::crospi_boost::type_traits::no_type is_class_tester(...);

template <typename T>
struct is_class_impl
{

    CROSPI_BOOST_STATIC_CONSTANT(bool, value =
            sizeof(is_class_tester<T>(0)) == sizeof(::crospi_boost::type_traits::yes_type)
            && ! ::crospi_boost::is_union<T>::value
        );
};

#else

template <typename T>
struct is_class_impl
{
    template <class U> static ::crospi_boost::type_traits::yes_type is_class_tester(void(U::*)(void));
    template <class U> static ::crospi_boost::type_traits::no_type is_class_tester(...);

    CROSPI_BOOST_STATIC_CONSTANT(bool, value =
            sizeof(is_class_tester<T>(0)) == sizeof(::crospi_boost::type_traits::yes_type)
            && ! ::crospi_boost::is_union<T>::value
        );
};

#endif

#else

template <typename T>
struct is_class_impl
{
    CROSPI_BOOST_STATIC_CONSTANT(bool, value =
        ! ::crospi_boost::is_union<T>::value >::value
        && ! ::crospi_boost::is_scalar<T>::value
        && ! ::crospi_boost::is_array<T>::value
        && ! ::crospi_boost::is_reference<T>::value
        && ! ::crospi_boost::is_void<T>::value
        && ! ::crospi_boost::is_function<T>::value
        );
};

# endif // CROSPI_BOOST_TT_HAS_CONFORMING_IS_CLASS_IMPLEMENTATION
# else // CROSPI_BOOST_IS_CLASS
template <typename T>
struct is_class_impl
{
    CROSPI_BOOST_STATIC_CONSTANT(bool, value = CROSPI_BOOST_IS_CLASS(T));
};
# endif // CROSPI_BOOST_IS_CLASS

} // namespace detail

template <class T> struct is_class : public integral_constant<bool, ::crospi_boost::detail::is_class_impl<T>::value> {};
# ifdef __EDG_VERSION__
template <class T> struct is_class<const T> : public is_class<T>{};
template <class T> struct is_class<const volatile T> : public is_class<T>{};
template <class T> struct is_class<volatile T> : public is_class<T>{};
# endif
    
} // namespace crospi_boost

#endif // CROSPI_BOOST_TT_IS_CLASS_HPP_INCLUDED
