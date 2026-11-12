// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_TAGGED_ARGUMENT_050328_HPP
#define CROSPI_BOOST_PARAMETER_TAGGED_ARGUMENT_050328_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    struct error_const_lvalue_bound_to_out_parameter;
    struct error_lvalue_bound_to_consume_parameter;
    struct error_rvalue_bound_to_out_parameter;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/keyword_fwd.hpp>
#include <crospi_boost/parameter/config.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#include <crospi_boost/type_traits/remove_const.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Keyword, typename Arg>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using tagged_argument_type = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_if<
            ::std::is_scalar<Arg>
          , ::crospi_boost::mp11::mp_false
          , ::std::is_same<
                typename Keyword::qualifier
              , ::crospi_boost::parameter::consume_reference
            >
        >
      , ::crospi_boost::parameter::aux::error_lvalue_bound_to_consume_parameter
      , ::crospi_boost::mp11::mp_if<
            ::std::is_const<Arg>
          , ::crospi_boost::mp11::mp_if<
                ::std::is_same<
                    typename Keyword::qualifier
                  , ::crospi_boost::parameter::out_reference
                >
              , ::crospi_boost::parameter::aux
                ::error_const_lvalue_bound_to_out_parameter
              , ::std::remove_const<Arg>
            >
          , ::crospi_boost::mp11::mp_identity<Arg>
        >
    >;
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    struct tagged_argument_type
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<
                typename Keyword::qualifier
              , ::crospi_boost::parameter::out_reference
            >
          , ::crospi_boost::parameter::aux::error_const_lvalue_bound_to_out_parameter
          , ::crospi_boost::remove_const<Arg>
        >
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/tagged_argument_fwd.hpp>
#include <crospi_boost/parameter/aux_/is_tagged_argument.hpp>
#include <crospi_boost/parameter/aux_/default.hpp>
#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/aux_/arg_list.hpp>
#include <crospi_boost/parameter/aux_/result_of0.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/identity.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/type_traits/is_const.hpp>
#include <crospi_boost/type_traits/is_function.hpp>
#include <crospi_boost/type_traits/is_scalar.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>

#if defined(CROSPI_BOOST_NO_CXX11_HDR_FUNCTIONAL)
#include <crospi_boost/function.hpp>
#else
#include <functional>
#endif

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#include <crospi_boost/core/enable_if.hpp>
#include <utility>

namespace crospi_boost { namespace parameter { namespace aux {

    // Holds an lvalue reference to an argument of type Arg associated with
    // keyword Keyword
    template <typename Keyword, typename Arg>
    class tagged_argument
      : public ::crospi_boost::parameter::aux::tagged_argument_base
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using arg_type = typename ::crospi_boost::parameter::aux
        ::tagged_argument_type<Keyword,Arg>::type;
#else
        typedef typename ::crospi_boost::mpl::eval_if<
            typename ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_scalar<Arg>
              , ::crospi_boost::mpl::false_
              , ::crospi_boost::is_same<
                    typename Keyword::qualifier
                  , ::crospi_boost::parameter::consume_reference
                >
            >::type
          , ::crospi_boost::parameter::aux::error_lvalue_bound_to_consume_parameter
          , ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_const<Arg>
              , ::crospi_boost::parameter::aux::tagged_argument_type<Keyword,Arg>
              , ::crospi_boost::mpl::identity<Arg>
            >
        >::type arg_type;
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

     public:
        typedef Keyword key_type;

        // Wrap plain (non-UDT) function objects in either
        // a crospi_boost::function or a std::function. -- Cromwell D. Enage
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using value_type = ::crospi_boost::mp11::mp_if<
            ::std::is_function<arg_type>
          , ::std::function<arg_type>
          , Arg
        >;
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
#if defined(CROSPI_BOOST_NO_CXX11_HDR_FUNCTIONAL)
          , ::crospi_boost::function<arg_type>
#else
          , ::std::function<arg_type>
#endif
          , Arg
        >::type value_type;
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

        // If Arg is void_, then this type will evaluate to void_&.  If the
        // supplied argument is a plain function, then this type will evaluate
        // to a reference-to-const function wrapper type.  If the supplied
        // argument is an lvalue, then Arg will be deduced to the lvalue
        // reference. -- Cromwell D. Enage
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using reference = ::crospi_boost::mp11::mp_if<
            ::std::is_function<arg_type>
          , value_type const&
          , Arg&
        >;
#else
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
          , value_type const&
          , Arg&
        >::type reference;
#endif

     private:
        // Store plain functions by value, everything else by reference.
        // -- Cromwell D. Enage
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        ::crospi_boost::mp11::mp_if<
            ::std::is_function<arg_type>
          , value_type
          , reference
        > value;
#else
        typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
          , value_type
          , reference
        >::type value;
#endif

     public:
        inline explicit CROSPI_BOOST_CONSTEXPR tagged_argument(reference x)
          : value(x)
        {
        }

        inline CROSPI_BOOST_CONSTEXPR tagged_argument(tagged_argument const& copy)
          : value(copy.value)
        {
        }

        // A metafunction class that, given a keyword and a default type,
        // returns the appropriate result type for a keyword lookup given
        // that default.
        struct binding
        {
            template <typename KW, typename Default, typename Reference>
            struct apply
              : ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::if_<Reference,reference,value_type>
                  , ::crospi_boost::mpl::identity<Default>
                >
            {
            };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            template <typename KW, typename Default, typename Reference>
            using fn = ::crospi_boost::mp11::mp_if<
                ::std::is_same<KW,key_type>
              , ::crospi_boost::mp11::mp_if<Reference,reference,value_type>
              , Default
            >;
#endif
        };

#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename Keyword2, typename Arg2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
          , ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
            >
        >
            operator,(
                ::crospi_boost::parameter::aux
                ::tagged_argument<Keyword2,Arg2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >
            >(
                *this
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >(x, ::crospi_boost::parameter::aux::empty_arg_list())
            );
        }

        template <typename Keyword2, typename Arg2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
          , ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword2,Arg2>
            >
        >
            operator,(
                ::crospi_boost::parameter::aux
                ::tagged_argument_rref<Keyword2,Arg2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
              , crospi_boost::parameter::aux::arg_list<
                    crospi_boost::parameter::aux::tagged_argument_rref<Keyword2,Arg2>
                >
            >(
                *this
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux
                    ::tagged_argument_rref<Keyword2,Arg2>
                >(x, ::crospi_boost::parameter::aux::empty_arg_list())
            );
        }
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

        // Accessor interface.
        inline CROSPI_BOOST_CONSTEXPR reference get_value() const
        {
            return this->value;
        }

        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](::crospi_boost::parameter::keyword<Keyword> const&) const
        {
            return this->get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_<key_type,Default> const&
            ) const
        {
            return this->get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_r_<key_type,Default> const&
            ) const
        {
            return this->get_value();
        }

        template <typename F>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<key_type,F> const&
            ) const
        {
            return this->get_value();
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            operator[](
                ::crospi_boost::parameter::aux::default_<KW,Default> const& x
            ) const
        {
            return x.value;
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&&
            operator[](
                ::crospi_boost::parameter::aux::default_r_<KW,Default> const& x
            ) const
        {
            return ::std::forward<Default>(x.value);
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<KW,F> const& x
            ) const
        {
            return x.compute_default();
        }

        template <typename ParameterRequirements>
        static CROSPI_BOOST_CONSTEXPR typename ParameterRequirements::has_default
            satisfies(ParameterRequirements*);

        template <typename HasDefault, typename Predicate>
        static CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::mpl::apply_wrap1<Predicate,value_type>::type
            satisfies(
                ::crospi_boost::parameter::aux::parameter_requirements<
                    key_type
                  , Predicate
                  , HasDefault
                >*
            );

        // MPL sequence support
        // Convenience for users
        typedef ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg> type;
        // For the benefit of iterators
        typedef ::crospi_boost::parameter::aux::empty_arg_list tail_type;
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    template <typename Keyword>
    using tagged_argument_rref_key = ::crospi_boost::mp11::mp_if<
        ::std::is_same<
            typename Keyword::qualifier
          , ::crospi_boost::parameter::out_reference
        >
      , ::crospi_boost::parameter::aux::error_rvalue_bound_to_out_parameter
      , ::crospi_boost::mp11::mp_identity<Keyword>
    >;
#endif

    // Holds an rvalue reference to an argument of type Arg associated with
    // keyword Keyword
    template <typename Keyword, typename Arg>
    struct tagged_argument_rref
      : ::crospi_boost::parameter::aux::tagged_argument_base
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using key_type = typename ::crospi_boost::parameter::aux
        ::tagged_argument_rref_key<Keyword>::type;
#else
        typedef typename ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<
                typename Keyword::qualifier
              , ::crospi_boost::parameter::out_reference
            >
          , ::crospi_boost::parameter::aux::error_rvalue_bound_to_out_parameter
          , ::crospi_boost::mpl::identity<Keyword>
        >::type key_type;
#endif
        typedef Arg value_type;
        typedef Arg&& reference;

     private:
        reference value;

     public:
        inline explicit CROSPI_BOOST_CONSTEXPR tagged_argument_rref(reference x)
          : value(::std::forward<Arg>(x))
        {
        }

        inline CROSPI_BOOST_CONSTEXPR tagged_argument_rref(
            tagged_argument_rref const& copy
        ) : value(::std::forward<Arg>(copy.value))
        {
        }

        // A metafunction class that, given a keyword and a default type,
        // returns the appropriate result type for a keyword lookup given
        // that default.
        struct binding
        {
            template <typename KW, typename Default, typename Reference>
            struct apply
            {
                typedef typename ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::if_<Reference,reference,value_type>
                  , ::crospi_boost::mpl::identity<Default>
                >::type type;
            };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            template <typename KW, typename Default, typename Reference>
            using fn = ::crospi_boost::mp11::mp_if<
                ::std::is_same<KW,key_type>
              , ::crospi_boost::mp11::mp_if<Reference,reference,value_type>
              , Default
            >;
#endif
        };

#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename Keyword2, typename Arg2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
          , ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
            >
        >
            operator,(
                ::crospi_boost::parameter::aux
                ::tagged_argument<Keyword2,Arg2> const& x
            ) const
        {
            return crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >
            >(
                *this
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >(x, ::crospi_boost::parameter::aux::empty_arg_list())
            );
        }

        template <typename Keyword2, typename Arg2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
          , ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword2,Arg2>
            >
        >
            operator,(
                ::crospi_boost::parameter::aux
                ::tagged_argument_rref<Keyword2,Arg2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument_rref<Keyword,Arg>
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux
                    ::tagged_argument_rref<Keyword2,Arg2>
                >
            >(
                *this
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument_rref<
                        Keyword2
                      , Arg2
                    >
                >(x, ::crospi_boost::parameter::aux::empty_arg_list())
            );
        }
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

        // Accessor interface.
        inline CROSPI_BOOST_CONSTEXPR reference get_value() const
        {
            return ::std::forward<Arg>(this->value);
        }

        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](::crospi_boost::parameter::keyword<Keyword> const&) const
        {
            return this->get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_<key_type,Default> const&
            ) const
        {
            return this->get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_r_<key_type,Default> const&
            ) const
        {
            return this->get_value();
        }

        template <typename F>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<key_type,F> const&
            ) const
        {
            return this->get_value();
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            operator[](
                ::crospi_boost::parameter::aux::default_<KW,Default> const& x
            ) const
        {
            return x.value;
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&&
            operator[](
                ::crospi_boost::parameter::aux::default_r_<KW,Default> const& x
            ) const
        {
            return ::std::forward<Default>(x.value);
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<KW,F> const& x
            ) const
        {
            return x.compute_default();
        }

        template <typename ParameterRequirements>
        static CROSPI_BOOST_CONSTEXPR typename ParameterRequirements::has_default
            satisfies(ParameterRequirements*);

        template <typename HasDefault, typename Predicate>
        static CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::mpl::apply_wrap1<Predicate,value_type>::type
            satisfies(
                ::crospi_boost::parameter::aux::parameter_requirements<
                    key_type
                  , Predicate
                  , HasDefault
                >*
            );

        // MPL sequence support
        // Convenience for users
        typedef ::crospi_boost::parameter::aux
        ::tagged_argument_rref<Keyword,Arg> type;
        // For the benefit of iterators
        typedef ::crospi_boost::parameter::aux::empty_arg_list tail_type;
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };
}}} // namespace crospi_boost::parameter::aux

#else   // !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

namespace crospi_boost { namespace parameter { namespace aux {

    // Holds an lvalue reference to an argument of type Arg associated with
    // keyword Keyword
    template <typename Keyword, typename Arg>
    class tagged_argument
      : public ::crospi_boost::parameter::aux::tagged_argument_base
    {
        typedef typename ::crospi_boost::remove_const<Arg>::type arg_type;

     public:
        typedef Keyword key_type;

        // Wrap plain (non-UDT) function objects in either
        // a crospi_boost::function or a std::function. -- Cromwell D. Enage
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
#if defined(CROSPI_BOOST_NO_CXX11_HDR_FUNCTIONAL)
          , ::crospi_boost::function<arg_type>
#else
          , ::std::function<arg_type>
#endif
          , Arg
        >::type value_type;

        // If Arg is void_, then this type will evaluate to void_&.  If the
        // supplied argument is a plain function, then this type will evaluate
        // to a reference-to-const function wrapper type.  If the supplied
        // argument is an lvalue, then Arg will be deduced to the lvalue
        // reference. -- Cromwell D. Enage
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
          , value_type const&
          , Arg&
        >::type reference;

     private:
        // Store plain functions by value, everything else by reference.
        // -- Cromwell D. Enage
        typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_function<arg_type>
          , value_type
          , reference
        >::type value;

     public:
        inline explicit CROSPI_BOOST_CONSTEXPR tagged_argument(reference x)
          : value(x)
        {
        }

        inline CROSPI_BOOST_CONSTEXPR tagged_argument(tagged_argument const& copy)
          : value(copy.value)
        {
        }

        // A metafunction class that, given a keyword and a default type,
        // returns the appropriate result type for a keyword lookup given
        // that default.
        struct binding
        {
            template <typename KW, typename Default, typename Reference>
            struct apply
            {
                typedef typename ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::if_<Reference,reference,value_type>
                  , ::crospi_boost::mpl::identity<Default>
                >::type type;
            };
        };

        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename Keyword2, typename Arg2>
        inline ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
          , ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
            >
        >
            operator,(
                ::crospi_boost::parameter::aux
                ::tagged_argument<Keyword2,Arg2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg>
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >
            >(
                *this
              , ::crospi_boost::parameter::aux::arg_list<
                    ::crospi_boost::parameter::aux::tagged_argument<Keyword2,Arg2>
                >(x, ::crospi_boost::parameter::aux::empty_arg_list())
            );
        }

        // Accessor interface.
        inline CROSPI_BOOST_CONSTEXPR reference get_value() const
        {
            return this->value;
        }

        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](::crospi_boost::parameter::keyword<Keyword> const&) const
        {
            return this->get_value();
        }

#if defined(CROSPI_BOOST_NO_FUNCTION_TEMPLATE_ORDERING) || \
    CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            get_with_default(
                ::crospi_boost::parameter::aux::default_<KW,Default> const& x
              , int
            ) const
        {
            return x.value;
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            get_with_default(
                ::crospi_boost::parameter::aux::default_<key_type,Default> const&
              , long
            ) const
        {
            return this->get_value();
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR typename ::crospi_boost::mpl::apply_wrap3<
            binding
          , KW
          , Default&
          , ::crospi_boost::mpl::true_
        >::type
            operator[](
                ::crospi_boost::parameter::aux::default_<KW,Default> const& x
            ) const
        {
            return this->get_with_default(x, 0L);
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            get_with_lazy_default(
                ::crospi_boost::parameter::aux::lazy_default<KW,F> const& x
              , int
            ) const
        {
            return x.compute_default();
        }

        template <typename F>
        inline CROSPI_BOOST_CONSTEXPR reference
            get_with_lazy_default(
                ::crospi_boost::parameter::aux::lazy_default<key_type,F> const&
              , long
            ) const
        {
            return this->get_value();
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR typename ::crospi_boost::mpl::apply_wrap3<
            binding
          , KW
          , typename ::crospi_boost::parameter::aux::result_of0<F>::type
          , ::crospi_boost::mpl::true_
        >::type
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<KW,F> const& x
            ) const
        {
            return this->get_with_lazy_default(x, 0L);
        }
#else   // No function template ordering or Borland workarounds needed.
        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_<key_type,Default> const&
            ) const
        {
            return this->get_value();
        }

        template <typename F>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<key_type,F> const&
            ) const
        {
            return this->get_value();
        }

        template <typename KW, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            operator[](
                ::crospi_boost::parameter::aux::default_<KW,Default> const& x
            ) const
        {
            return x.value;
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            operator[](
                ::crospi_boost::parameter::aux::lazy_default<KW,F> const& x
            ) const
        {
            return x.compute_default();
        }

        template <typename ParameterRequirements>
        static CROSPI_BOOST_CONSTEXPR typename ParameterRequirements::has_default
            satisfies(ParameterRequirements*);

        template <typename HasDefault, typename Predicate>
        static CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::mpl::apply_wrap1<Predicate,value_type>::type
            satisfies(
                ::crospi_boost::parameter::aux::parameter_requirements<
                    key_type
                  , Predicate
                  , HasDefault
                >*
            );
#endif  // Function template ordering, Borland workarounds needed.

        // MPL sequence support
        // Convenience for users
        typedef ::crospi_boost::parameter::aux::tagged_argument<Keyword,Arg> type;
        // For the benefit of iterators
        typedef ::crospi_boost::parameter::aux::empty_arg_list tail_type;
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, CROSPI_BOOST_TESTED_AT(1310))
        // warning suppression
     private:
        void operator=(type const&);
#endif
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename TaggedArg>
    struct tagged_argument_list_of_1 : public TaggedArg
    {
        using base_type = TaggedArg;

        inline explicit CROSPI_BOOST_CONSTEXPR tagged_argument_list_of_1(
            typename base_type::reference x
        ) : base_type(static_cast<typename base_type::reference>(x))
        {
        }

        inline CROSPI_BOOST_CONSTEXPR tagged_argument_list_of_1(
            tagged_argument_list_of_1 const& copy
        ) : base_type(static_cast<base_type const&>(copy))
        {
        }

        using base_type::operator[];
        using base_type::satisfies;

        template <typename TA2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::flat_like_arg_list<
            ::crospi_boost::parameter::aux
            ::flat_like_arg_tuple<typename TaggedArg::key_type,TaggedArg>
          , ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                typename TA2::base_type::key_type
              , typename TA2::base_type
            >
        >
            operator,(TA2 const& x) const
        {
            return crospi_boost::parameter::aux::flat_like_arg_list<
                ::crospi_boost::parameter::aux
                ::flat_like_arg_tuple<typename TaggedArg::key_type,TaggedArg>
              , ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                    typename TA2::base_type::key_type
                  , typename TA2::base_type
                >
            >(
                static_cast<base_type const&>(*this)
              , ::crospi_boost::parameter::aux::arg_list<typename TA2::base_type>(
                    static_cast<typename TA2::base_type const&>(x)
                  , ::crospi_boost::parameter::aux::empty_arg_list()
                )
            );
        }
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
#endif  // include guard

