// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_UNWRAP_CV_REFERENCE_HPP
#define CROSPI_BOOST_PARAMETER_AUX_UNWRAP_CV_REFERENCE_HPP

namespace crospi_boost {

    template <typename T>
    class reference_wrapper;
} // namespace crospi_boost

#include <crospi_boost/parameter/aux_/yesno.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    //
    // reference_wrapper support -- if perfect forwarding is unsupported,
    // then when passing arguments positionally by non-const reference,
    // we ask users of named parameter interfaces to use ref(x) to wrap them.
    //

    template <typename U>
    ::crospi_boost::parameter::aux::yes_tag
        is_cv_reference_wrapper_check(
            ::crospi_boost::reference_wrapper<U> const volatile*
        );

    ::crospi_boost::parameter::aux::no_tag is_cv_reference_wrapper_check(...);
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if !defined(CROSPI_BOOST_NO_CXX11_HDR_FUNCTIONAL)
#include <functional>

namespace crospi_boost { namespace parameter { namespace aux {

    // Support for std::ref(x) -- Cromwell D. Enage
    template <typename U>
    ::crospi_boost::parameter::aux::yes_tag
        is_cv_reference_wrapper_check(
            ::std::reference_wrapper<U> const volatile*
        );
}}} // namespace crospi_boost::parameter::aux
#endif

#include <crospi_boost/parameter/aux_/preprocessor/nullptr.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) && !( \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, >= 1900) && \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1910) \
    )
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) || MSVC-14.0
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>
#if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564)) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 40000)
#include <crospi_boost/mpl/eval_if.hpp>
#endif
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11 && not MSVC-14.0

namespace crospi_boost { namespace parameter { namespace aux {

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) && !( \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, >= 1900) && \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1910) \
    )
    // This metafunction returns mp11::mp_true if T is of type
    // reference_wrapper<U> cv.
    template <typename T>
    using is_cv_reference_wrapper = ::crospi_boost::mp11::mp_bool<
        sizeof(
            ::crospi_boost::parameter::aux::is_cv_reference_wrapper_check(
                static_cast<
                    typename ::std::remove_reference<T>::type*
                >(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
            )
        ) == sizeof(::crospi_boost::parameter::aux::yes_tag)
    >;

    // Needed for unwrap_cv_reference below. T might be const, so
    // mp_eval_if<> might fail because of deriving from T const on EDG.
    template <typename T>
    using unwrap_cv_reference_impl = typename ::std::remove_reference<T>::type;

    // Produces the unwrapped type to hold a reference to in
    // tagged_argument<>.  Can't use crospi_boost::unwrap_reference<> here
    // because it doesn't handle the case where T = reference_wrapper<U> cv.
    template <typename T>
    using unwrap_cv_reference = ::crospi_boost::mp11::mp_eval_if<
        ::crospi_boost::parameter::aux::is_cv_reference_wrapper<T>
      , ::crospi_boost::parameter::aux::unwrap_cv_reference_impl<T>
      , ::std::remove_reference
      , T
    >;
#else  // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11) || MSVC-14.0
    // This metafunction returns mpl::true_ if T is of type
    // reference_wrapper<U> cv.
    template <typename T>
    struct is_cv_reference_wrapper
    {
        CROSPI_BOOST_STATIC_CONSTANT(
            bool, value = (
                sizeof(
                    ::crospi_boost::parameter::aux::is_cv_reference_wrapper_check(
                        static_cast<
                            typename ::crospi_boost::remove_reference<T>::type*
                        >(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                    )
                ) == sizeof(::crospi_boost::parameter::aux::yes_tag)
            )
        );

        typedef crospi_boost::mpl::bool_<
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
            is_cv_reference_wrapper::
#endif
        value> type;
    };

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564)) || \
    CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 40000)
    template <
        typename T
      , typename = typename ::crospi_boost::parameter::aux
        ::is_cv_reference_wrapper<T>::type
    >
    struct unwrap_cv_reference : ::crospi_boost::remove_reference<T>
    {
    };

    template <typename T>
    struct unwrap_cv_reference<T const,::crospi_boost::mpl::false_>
    {
        typedef T const type;
    };

    template <typename T>
    struct unwrap_cv_reference<T,::crospi_boost::mpl::true_> : T
    {
    };
#else   // no Borland or GCC 3- workarounds needed
    // Needed for unwrap_cv_reference below. T might be const, so
    // eval_if<> might fail because of deriving from T const on EDG.
    template <typename T>
    struct unwrap_cv_reference_impl : ::crospi_boost::remove_reference<T>::type
    {
    };

    // Produces the unwrapped type to hold a reference to in
    // tagged_argument<>.  Can't use crospi_boost::unwrap_reference<> here
    // because it doesn't handle the case where T = reference_wrapper<U> cv.
    template <typename T>
    struct unwrap_cv_reference
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::parameter::aux::is_cv_reference_wrapper<T>
          , ::crospi_boost::parameter::aux::unwrap_cv_reference_impl<T>
          , ::crospi_boost::remove_reference<T>
        >
    {
    };
#endif  // Borland or GCC 3- workarounds needed
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11 && not MSVC-14.0
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

