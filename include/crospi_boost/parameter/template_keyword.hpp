// Copyright Daniel Wallin 2006.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_TEMPLATE_KEYWORD_HPP
#define CROSPI_BOOST_PARAMETER_TEMPLATE_KEYWORD_HPP

#include <crospi_boost/parameter/aux_/template_keyword.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/type_traits/add_lvalue_reference.hpp>
#include <crospi_boost/type_traits/is_function.hpp>
#include <crospi_boost/type_traits/is_array.hpp>
#endif

namespace crospi_boost { namespace parameter {

    template <typename Tag, typename T>
    struct template_keyword : ::crospi_boost::parameter::aux::template_keyword_base
    {
        typedef Tag key_type;
        typedef T value_type;

        // reference is needed for two reasons:
        //
        // 1. It is used in the body of arg_list<...>
        //
        // 2. It is the result of binding<...>, which we mistakenly told
        //    people to use instead of value_type<...> to access named
        //    template parameters
        //
        // It used to be that reference == value_type, but that broke when
        // the argument was a function or array type, because various
        // arg_list functions return reference.
        //
        // Simply making reference == value_type& would break all the
        // legacy code that uses binding<...> to access named template
        // parameters. -- David Abrahams
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using reference = typename ::crospi_boost::mp11::mp_eval_if<
            ::crospi_boost::mp11::mp_if<
                ::std::is_function<value_type>
              , ::crospi_boost::mp11::mp_true
              , ::std::is_array<value_type>
            >
          , ::std::add_lvalue_reference<value_type>
          , ::crospi_boost::mp11::mp_identity
          , value_type
        >::type;
#else
        typedef typename ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::is_function<value_type>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::is_array<value_type>
            >::type
          , ::crospi_boost::add_lvalue_reference<value_type>
          , ::crospi_boost::mpl::identity<value_type>
        >::type reference;
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
    };
}} // namespace crospi_boost::parameter

#define CROSPI_BOOST_PARAMETER_TEMPLATE_KEYWORD(name)                               \
    namespace tag                                                            \
    {                                                                        \
        struct name;                                                         \
    }                                                                        \
    template <typename T>                                                    \
    struct name : ::crospi_boost::parameter::template_keyword<tag::name,T>          \
    {                                                                        \
    };
/**/

#endif  // include guard

