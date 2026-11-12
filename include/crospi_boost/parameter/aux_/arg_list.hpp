// Copyright Daniel Wallin, David Abrahams 2005.
// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_ARG_LIST_HPP
#define CROSPI_BOOST_PARAMETER_AUX_ARG_LIST_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    //
    // Structures used to build the tuple of actual arguments.  The tuple is a
    // nested cons-style list of arg_list specializations terminated by an
    // empty_arg_list.
    //
    // Each specialization of arg_list is derived from its successor in the
    // list type.  This feature is used along with using declarations to build
    // member function overload sets that can match against keywords.
    //

    // MPL sequence support
    struct arg_list_tag;

    template <typename T>
    struct get_reference
    {
        typedef typename T::reference type;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

namespace crospi_boost { namespace parameter { namespace aux {

    struct value_type_is_void
    {
    };

    struct value_type_is_not_void
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/parameter/aux_/yesno.hpp>
#include <crospi_boost/parameter/aux_/result_of0.hpp>
#include <crospi_boost/parameter/aux_/default.hpp>

#if defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#include <utility>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/list.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // Terminates arg_list<> and represents an empty list.  Since this is just
    // the terminating case, you might want to look at arg_list first to get a
    // feel for what's really happening here.
    struct empty_arg_list
    {
        struct tagged_arg
        {
            typedef ::crospi_boost::parameter::void_ value_type;
        };

        // Variadic constructor also serves as default constructor.
        template <typename ...Args>
        inline CROSPI_BOOST_CONSTEXPR empty_arg_list(Args&&...)
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
                typedef Default type;
            };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            template <typename KW, typename Default, typename Reference>
            using fn = Default;
#endif
        };

        // Terminator for has_key, indicating that the keyword is unique.
        template <typename KW>
        static ::crospi_boost::parameter::aux::no_tag has_key(KW*);

        // If either of these operators are called, it means there is no
        // argument in the list that matches the supplied keyword.  Just
        // return the default value.
        template <typename K, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            operator[](::crospi_boost::parameter::aux::default_<K,Default> x) const
        {
            return x.value;
        }

        template <typename K, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&&
            operator[](::crospi_boost::parameter::aux::default_r_<K,Default> x) const
        {
            return ::std::forward<Default>(x.value);
        }

        // If this operator is called, it means there is no argument in the
        // list that matches the supplied keyword.  Just evaluate and return
        // the default value.
        template <typename K, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            operator[](CROSPI_BOOST_PARAMETER_lazy_default_fallback<K,F> x) const
        {
            return x.compute_default();
        }

        // No argument corresponding to ParameterRequirements::key_type
        // was found if we match this overload, so unless that parameter
        // has a default, we indicate that the actual arguments don't
        // match the function's requirements.
        template <typename ParameterRequirements, typename ArgPack>
        static typename ParameterRequirements::has_default
            satisfies(ParameterRequirements*, ArgPack*);

        // MPL sequence support
        typedef ::crospi_boost::parameter::aux::empty_arg_list type; // convenience
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/preprocessor/nullptr.hpp>
#include <crospi_boost/parameter/aux_/yesno.hpp>
#include <crospi_boost/parameter/aux_/is_maybe.hpp>
#include <crospi_boost/parameter/aux_/tagged_argument_fwd.hpp>
#include <crospi_boost/parameter/aux_/parameter_requirements.hpp>
#include <crospi_boost/parameter/aux_/augment_predicate.hpp>
#include <crospi_boost/parameter/keyword_fwd.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/assert.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#include <crospi_boost/core/enable_if.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // A tuple of tagged arguments, terminated with empty_arg_list.  Every
    // TaggedArg is an instance of tagged_argument<> or
    // tagged_argument_rref<>.
    template <
        typename TaggedArg
      , typename Next = ::crospi_boost::parameter::aux::empty_arg_list
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      , typename EmitsErrors = ::crospi_boost::mp11::mp_true
#else
      , typename EmitsErrors = ::crospi_boost::mpl::true_
#endif
    >
    class arg_list : public Next
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using _holds_maybe = typename ::crospi_boost::parameter::aux
        ::is_maybe<typename TaggedArg::value_type>::type;
#else
        typedef typename ::crospi_boost::parameter::aux
        ::is_maybe<typename TaggedArg::value_type>::type _holds_maybe;
#endif

        TaggedArg arg;      // Stores the argument

     public:
        typedef TaggedArg tagged_arg;
        typedef ::crospi_boost::parameter::aux::arg_list<TaggedArg,Next> self;
        typedef typename TaggedArg::key_type key_type;

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using reference = typename ::crospi_boost::mp11::mp_if<
            _holds_maybe
          , ::crospi_boost::parameter::aux
            ::get_reference<typename TaggedArg::value_type>
          , ::crospi_boost::parameter::aux::get_reference<TaggedArg>
        >::type;

        using value_type = ::crospi_boost::mp11
        ::mp_if<_holds_maybe,reference,typename TaggedArg::value_type>;
#else   // !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        typedef typename ::crospi_boost::mpl::eval_if<
            _holds_maybe
          , ::crospi_boost::parameter::aux
            ::get_reference<typename TaggedArg::value_type>
          , ::crospi_boost::parameter::aux::get_reference<TaggedArg>
        >::type reference;

        typedef typename ::crospi_boost::mpl::if_<
            _holds_maybe
          , reference
          , typename TaggedArg::value_type
        >::type value_type;
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

        // Create a new list by prepending arg to a copy of tail.  Used when
        // incrementally building this structure with the comma operator.
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            TaggedArg const& head
          , Next const& tail
        ) : Next(tail), arg(head)
        {
        }

        // Store the arguments in successive nodes of this list.
        // Use tag dispatching to determine whether to forward all arguments
        // to the Next constructor, or store the first argument and forward
        // the rest. -- Cromwell D. Enage
        template <typename A0>
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            ::crospi_boost::parameter::aux::value_type_is_not_void
          , A0&& a0
        ) : Next(
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_if<
                    ::std::is_same<
#else
                typename ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_same<
#endif
                        typename Next::tagged_arg::value_type
                      , ::crospi_boost::parameter::void_
                    >
                  , ::crospi_boost::parameter::aux::value_type_is_void
                  , ::crospi_boost::parameter::aux::value_type_is_not_void
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                >()
#else
                >::type()
#endif
            )
          , arg(::std::forward<A0>(a0))
        {
        }

        template <typename ...Args>
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            ::crospi_boost::parameter::aux::value_type_is_void
          , Args&&... args
        ) : Next(
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_if<
                    ::std::is_same<
#else
                typename ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_same<
#endif
                        typename Next::tagged_arg::value_type
                      , ::crospi_boost::parameter::void_
                    >
                  , ::crospi_boost::parameter::aux::value_type_is_void
                  , ::crospi_boost::parameter::aux::value_type_is_not_void
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                >()
#else
                >::type()
#endif
              , ::std::forward<Args>(args)...
            )
          , arg(::crospi_boost::parameter::aux::void_reference())
        {
        }

        template <typename A0, typename A1, typename ...Args>
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            ::crospi_boost::parameter::aux::value_type_is_not_void
          , A0&& a0
          , A1&& a1
          , Args&&... args
        ) : Next(
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_if<
                    ::std::is_same<
#else
                typename ::crospi_boost::mpl::if_<
                    ::crospi_boost::is_same<
#endif
                        typename Next::tagged_arg::value_type
                      , ::crospi_boost::parameter::void_
                    >
                  , ::crospi_boost::parameter::aux::value_type_is_void
                  , ::crospi_boost::parameter::aux::value_type_is_not_void
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                >()
#else
                >::type()
#endif
              , ::std::forward<A1>(a1)
              , ::std::forward<Args>(args)...
            )
          , arg(::std::forward<A0>(a0))
        {
        }

        // A metafunction class that, given a keyword and a default type,
        // returns the appropriate result type for a keyword lookup given
        // that default.
        struct binding
        {
            typedef typename Next::binding next_binding;

            template <typename KW, typename Default, typename Reference>
            struct apply
            {
                typedef typename ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::if_<Reference,reference,value_type>
                  , ::crospi_boost::mpl
                    ::apply_wrap3<next_binding,KW,Default,Reference>
                >::type type;
            };

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            template <typename KW, typename Default, typename Reference>
            using fn = ::crospi_boost::mp11::mp_if<
                ::std::is_same<KW,key_type>
              , ::crospi_boost::mp11::mp_if<Reference,reference,value_type>
              , ::crospi_boost::mp11::mp_apply_q<
                    next_binding
                  , ::crospi_boost::mp11::mp_list<KW,Default,Reference>
                >
            >;
#endif
        };

        // Overload for key_type, so the assert below will fire
        // if the same keyword is used again.
        static ::crospi_boost::parameter::aux::yes_tag has_key(key_type*);
        using Next::has_key;

     private:
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using _has_unique_key = ::crospi_boost::mp11::mp_bool<
#else
        typedef ::crospi_boost::mpl::bool_<
#endif
            sizeof(
                Next::has_key(
                    static_cast<key_type*>(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                )
            ) == sizeof(::crospi_boost::parameter::aux::no_tag)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        >;
#else
        > _has_unique_key;
#endif

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        static_assert(
            !(EmitsErrors::value) || (_has_unique_key::value)
          , "duplicate keyword"
        );
#else
        CROSPI_BOOST_MPL_ASSERT_MSG(
            !(EmitsErrors::value) || (_has_unique_key::value)
          , duplicate_keyword
          , (key_type)
        );
#endif

        //
        // Begin implementation of indexing operators
        // for looking up specific arguments by name.
        //

        // Helpers that handle the case when TaggedArg is empty<T>.
        template <typename D>
        inline CROSPI_BOOST_CONSTEXPR reference
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            get_default(D const&, ::crospi_boost::mp11::mp_false) const
#else
            get_default(D const&, ::crospi_boost::mpl::false_) const
#endif
        {
            return this->arg.get_value();
        }

        template <typename D>
        inline CROSPI_BOOST_CONSTEXPR reference
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            get_default(D const& d, ::crospi_boost::mp11::mp_true) const
#else
            get_default(D const& d, ::crospi_boost::mpl::true_) const
#endif
        {
            return (
                this->arg.get_value()
              ? this->arg.get_value().get()
              : this->arg.get_value().construct(d.value)
            );
        }

     public:
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](::crospi_boost::parameter::keyword<key_type> const&) const
        {
#if !defined(CROSPI_BOOST_NO_CXX14_CONSTEXPR)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            static_assert(!_holds_maybe::value, "must not hold maybe");
#elif !( \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, >= 40700) && \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 40900) \
    ) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, >= 50000) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1910)
            CROSPI_BOOST_MPL_ASSERT_NOT((_holds_maybe));
#endif
#endif
            return this->arg.get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_<key_type,Default> const& d
            ) const
        {
            return this->get_default(d, _holds_maybe());
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_r_<key_type,Default> const& d
            ) const
        {
            return this->get_default(d, _holds_maybe());
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                CROSPI_BOOST_PARAMETER_lazy_default_fallback<key_type,Default> const&
            ) const
        {
#if !defined(CROSPI_BOOST_NO_CXX14_CONSTEXPR)
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            static_assert(!_holds_maybe::value, "must not hold maybe");
#elif !( \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, >= 40700) && \
        CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, < 40900) \
    ) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_GCC, >= 50000) && \
    !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1910)
            CROSPI_BOOST_MPL_ASSERT_NOT((_holds_maybe));
#endif
#endif
            return this->arg.get_value();
        }

        // Builds an overload set including operator[]s defined
        // in base classes.
        using Next::operator[];

        //
        // End of indexing support
        //

        // For parameter_requirements matching this node's key_type, return
        // a bool constant wrapper indicating whether the requirements are
        // satisfied by TaggedArg.  Used only for compile-time computation
        // and never really called, so a declaration is enough.
        template <typename HasDefault, typename Predicate, typename ArgPack>
        static typename ::crospi_boost::lazy_enable_if<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            ::crospi_boost::mp11::mp_if<
                EmitsErrors
              , ::crospi_boost::mp11::mp_true
              , _has_unique_key
            >
          , ::crospi_boost::parameter::aux::augment_predicate_mp11<
#else
            typename ::crospi_boost::mpl::if_<
                EmitsErrors
              , ::crospi_boost::mpl::true_
              , _has_unique_key
            >::type
          , ::crospi_boost::parameter::aux::augment_predicate<
#endif
                Predicate
              , reference
              , key_type
              , value_type
              , ArgPack
            >
        >::type
            satisfies(
                ::crospi_boost::parameter::aux::parameter_requirements<
                    key_type
                  , Predicate
                  , HasDefault
                >*
              , ArgPack*
            );

        // Builds an overload set including satisfies functions defined
        // in base classes.
        using Next::satisfies;

        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename KW, typename T2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument<KW,T2>
          , self
        >
            operator,(
                ::crospi_boost::parameter::aux::tagged_argument<KW,T2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<KW,T2>
              , self
            >(x, *this);
        }

        template <typename KW, typename T2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument_rref<KW,T2>
          , self
        >
            operator,(
                ::crospi_boost::parameter::aux::tagged_argument_rref<KW,T2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument_rref<KW,T2>
              , self
            >(x, *this);
        }

        // MPL sequence support
        typedef self type;        // Convenience for users
        typedef Next tail_type;   // For the benefit of iterators
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };
}}} // namespace crospi_boost::parameter::aux

#else   // !defined(CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

#include <crospi_boost/preprocessor/repetition/enum_params.hpp>
#include <crospi_boost/preprocessor/facilities/intercept.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // Terminates arg_list<> and represents an empty list.  Since this is just
    // the terminating case, you might want to look at arg_list first to get a
    // feel for what's really happening here.
    struct empty_arg_list
    {
        inline CROSPI_BOOST_CONSTEXPR empty_arg_list()
        {
        }

        // Constructor taking CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY empty_arg_list
        // arguments; this makes initialization.
        inline CROSPI_BOOST_CONSTEXPR empty_arg_list(
            CROSPI_BOOST_PP_ENUM_PARAMS(
                CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY
              , ::crospi_boost::parameter::void_ CROSPI_BOOST_PP_INTERCEPT
            )
        )
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
                typedef Default type;
            };
        };

        // Terminator for has_key, indicating that the keyword is unique.
        template <typename KW>
        static ::crospi_boost::parameter::aux::no_tag has_key(KW*);

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        // The overload set technique doesn't work with these older compilers,
        // so they need some explicit handholding.

        // A metafunction class that, given a keyword, returns the type of the
        // base sublist whose get() function can produce the value for that key.
        struct key_owner
        {
            template <typename KW>
            struct apply
            {
                typedef ::crospi_boost::parameter::aux::empty_arg_list type;
            };
        };
#endif  // Borland workarounds needed

        // If either of these operators are called, it means there is no
        // argument in the list that matches the supplied keyword.  Just
        // return the default value.
        template <typename K, typename Default>
        inline CROSPI_BOOST_CONSTEXPR Default&
            operator[](::crospi_boost::parameter::aux::default_<K,Default> x) const
        {
            return x.value;
        }

        // If this operator is called, it means there is no argument in the
        // list that matches the supplied keyword.  Just evaluate and return
        // the default value.
        template <typename K, typename F>
        inline CROSPI_BOOST_CONSTEXPR
        typename ::crospi_boost::parameter::aux::result_of0<F>::type
            operator[](CROSPI_BOOST_PARAMETER_lazy_default_fallback<K,F> x) const
        {
            return x.compute_default();
        }

        // No argument corresponding to ParameterRequirements::key_type
        // was found if we match this overload, so unless that parameter
        // has a default, we indicate that the actual arguments don't
        // match the function's requirements.
        template <typename ParameterRequirements, typename ArgPack>
        static typename ParameterRequirements::has_default
            satisfies(ParameterRequirements*, ArgPack*);

        // MPL sequence support
        typedef ::crospi_boost::parameter::aux::empty_arg_list type; // convenience
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/yesno.hpp>
#include <crospi_boost/parameter/aux_/is_maybe.hpp>
#include <crospi_boost/parameter/aux_/tagged_argument_fwd.hpp>
#include <crospi_boost/parameter/aux_/parameter_requirements.hpp>
#include <crospi_boost/parameter/aux_/augment_predicate.hpp>
#include <crospi_boost/parameter/keyword_fwd.hpp>
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/assert.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#include <crospi_boost/preprocessor/repetition/enum_binary_params.hpp>
#include <crospi_boost/preprocessor/repetition/enum_shifted_params.hpp>

#if !defined(CROSPI_BOOST_NO_SFINAE) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1800)
#include <crospi_boost/core/enable_if.hpp>
#endif

#if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
#include <crospi_boost/parameter/aux_/preprocessor/nullptr.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // A tuple of tagged arguments, terminated with empty_arg_list.  Every
    // TaggedArg is an instance of tagged_argument<>.
    template <
        typename TaggedArg
      , typename Next = ::crospi_boost::parameter::aux::empty_arg_list
      , typename EmitsErrors = ::crospi_boost::mpl::true_
    >
    class arg_list : public Next
    {
        typedef typename ::crospi_boost::parameter::aux
        ::is_maybe<typename TaggedArg::value_type>::type _holds_maybe;

        TaggedArg arg;      // Stores the argument

     public:
        typedef TaggedArg tagged_arg;
        typedef ::crospi_boost::parameter::aux::arg_list<TaggedArg,Next> self;
        typedef typename TaggedArg::key_type key_type;

        typedef typename ::crospi_boost::mpl::eval_if<
            _holds_maybe
          , ::crospi_boost::parameter::aux
            ::get_reference<typename TaggedArg::value_type>
          , ::crospi_boost::parameter::aux::get_reference<TaggedArg>
        >::type reference;

        typedef typename ::crospi_boost::mpl::if_<
            _holds_maybe
          , reference
          , typename TaggedArg::value_type
        >::type value_type;

        // Create a new list by prepending arg to a copy of tail.  Used when
        // incrementally building this structure with the comma operator.
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            TaggedArg const& head
          , Next const& tail
        ) : Next(tail), arg(head)
        {
        }

        // Store the arguments in successive nodes of this list.
        template <
            // typename A0, typename A1, ...
            CROSPI_BOOST_PP_ENUM_PARAMS(
                CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY
              , typename A
            )
        >
        inline CROSPI_BOOST_CONSTEXPR arg_list(
            // A0& a0, A1& a1, ...
            CROSPI_BOOST_PP_ENUM_BINARY_PARAMS(
                CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY
              , A
              , & a
            )
        ) : Next(
                // a1, a2, ...
                CROSPI_BOOST_PP_ENUM_SHIFTED_PARAMS(
                    CROSPI_BOOST_PARAMETER_COMPOSE_MAX_ARITY
                  , a
                )
              , ::crospi_boost::parameter::aux::void_reference()
            )
          , arg(a0)
        {
        }

        // A metafunction class that, given a keyword and a default type,
        // returns the appropriate result type for a keyword lookup given
        // that default.
        struct binding
        {
            typedef typename Next::binding next_binding;

            template <typename KW, typename Default, typename Reference>
            struct apply
            {
                typedef typename ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::if_<Reference,reference,value_type>
                  , ::crospi_boost::mpl::apply_wrap3<
                        next_binding
                      , KW
                      , Default
                      , Reference
                    >
                >::type type;
            };
        };

#if !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        // Overload for key_type, so the assert below will fire
        // if the same keyword is used again.
        static ::crospi_boost::parameter::aux::yes_tag has_key(key_type*);
        using Next::has_key;

     private:
#if defined(CROSPI_BOOST_NO_SFINAE) || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1800)
        CROSPI_BOOST_MPL_ASSERT_MSG(
            sizeof(
                Next::has_key(
                    static_cast<key_type*>(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                )
            ) == sizeof(::crospi_boost::parameter::aux::no_tag)
          , duplicate_keyword
          , (key_type)
        );
#else
        typedef ::crospi_boost::mpl::bool_<
            sizeof(
                Next::has_key(
                    static_cast<key_type*>(CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR)
                )
            ) == sizeof(::crospi_boost::parameter::aux::no_tag)
        > _has_unique_key;

        CROSPI_BOOST_MPL_ASSERT_MSG(
            !(EmitsErrors::value) || (_has_unique_key::value)
          , duplicate_keyword
          , (key_type)
        );
#endif  // SFINAE/MSVC workarounds needed
#endif  // Borland workarounds not needed

     private:
        //
        // Begin implementation of indexing operators
        // for looking up specific arguments by name.
        //

        // Helpers that handle the case when TaggedArg is empty<T>.
        template <typename D>
        inline CROSPI_BOOST_CONSTEXPR reference
            get_default(D const&, ::crospi_boost::mpl::false_) const
        {
            return this->arg.get_value();
        }

        template <typename D>
        inline CROSPI_BOOST_CONSTEXPR reference
            get_default(D const& d, ::crospi_boost::mpl::true_) const
        {
            return (
                this->arg.get_value()
              ? this->arg.get_value().get()
              : this->arg.get_value().construct(d.value)
            );
        }

     public:
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        // These older compilers don't support the overload set creation
        // idiom well, so we need to do all the return type calculation
        // for the compiler and dispatch through an outer function template.

        // A metafunction class that, given a keyword, returns the base
        // sublist whose get() function can produce the value for that key.
        struct key_owner
        {
            typedef typename Next::key_owner next_key_owner;

            template <typename KW>
            struct apply
            {
                typedef typename ::crospi_boost::mpl::eval_if<
                    ::crospi_boost::is_same<KW,key_type>
                  , ::crospi_boost::mpl::identity<
                        ::crospi_boost::parameter::aux::arg_list<TaggedArg,Next>
                    >
                  , ::crospi_boost::mpl::apply_wrap1<next_key_owner,KW>
                >::type type;
            };
        };

        // Outer indexing operators that dispatch to the right node's
        // get() function.
        template <typename KW>
        inline CROSPI_BOOST_CONSTEXPR typename ::crospi_boost::mpl::apply_wrap3<
            binding
          , KW
          , ::crospi_boost::parameter::void_
          , ::crospi_boost::mpl::true_
        >::type
            operator[](::crospi_boost::parameter::keyword<KW> const& x) const
        {
            typename ::crospi_boost::mpl::apply_wrap1<key_owner,KW>::type const&
                sublist = *this;
            return sublist.get(x);
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
            typename ::crospi_boost::mpl::apply_wrap1<key_owner,KW>::type const&
                sublist = *this;
            return sublist.get(x);
        }

        template <typename KW, typename F>
        inline CROSPI_BOOST_CONSTEXPR typename ::crospi_boost::mpl::apply_wrap3<
            binding
          , KW
          , typename ::crospi_boost::parameter::aux::result_of0<F>::type
          , ::crospi_boost::mpl::true_
        >::type
            operator[](
                CROSPI_BOOST_PARAMETER_lazy_default_fallback<KW,F> const& x
            ) const
        {
            typename ::crospi_boost::mpl::apply_wrap1<key_owner,KW>::type const&
                sublist = *this;
            return sublist.get(x);
        }

        // These just return the stored value; when empty_arg_list is reached,
        // indicating no matching argument was passed, the default is
        // returned, or if no default_ or lazy_default was passed, compilation
        // fails.
        inline CROSPI_BOOST_CONSTEXPR reference
            get(::crospi_boost::parameter::keyword<key_type> const&) const
        {
            CROSPI_BOOST_MPL_ASSERT_NOT((_holds_maybe));
            return this->arg.get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            get(
                ::crospi_boost::parameter::aux::default_<key_type,Default> const& d
            ) const
        {
            return this->get_default(d, _holds_maybe());
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            get(
                CROSPI_BOOST_PARAMETER_lazy_default_fallback<key_type,Default> const&
            ) const
        {
            return this->arg.get_value();
        }
#else   // !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](::crospi_boost::parameter::keyword<key_type> const&) const
        {
            CROSPI_BOOST_MPL_ASSERT_NOT((_holds_maybe));
            return this->arg.get_value();
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                ::crospi_boost::parameter::aux::default_<key_type,Default> const& d
            ) const
        {
            return this->get_default(d, _holds_maybe());
        }

        template <typename Default>
        inline CROSPI_BOOST_CONSTEXPR reference
            operator[](
                CROSPI_BOOST_PARAMETER_lazy_default_fallback<key_type,Default> const&
            ) const
        {
            CROSPI_BOOST_MPL_ASSERT_NOT((_holds_maybe));
            return this->arg.get_value();
        }

        // Builds an overload set including operator[]s defined
        // in base classes.
        using Next::operator[];

        //
        // End of indexing support
        //

        // For parameter_requirements matching this node's key_type, return
        // a bool constant wrapper indicating whether the requirements are
        // satisfied by TaggedArg.  Used only for compile-time computation
        // and never really called, so a declaration is enough.
        template <typename HasDefault, typename Predicate, typename ArgPack>
        static typename
#if !defined(CROSPI_BOOST_NO_SFINAE) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1800)
        ::crospi_boost::lazy_enable_if<
            typename ::crospi_boost::mpl::if_<
                EmitsErrors
              , ::crospi_boost::mpl::true_
              , _has_unique_key
            >::type,
#endif
            ::crospi_boost::parameter::aux::augment_predicate<
                Predicate
              , reference
              , key_type
              , value_type
              , ArgPack
#if !defined(CROSPI_BOOST_NO_SFINAE) && !CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1800)
           >
#endif
        >::type
            satisfies(
                ::crospi_boost::parameter::aux::parameter_requirements<
                    key_type
                  , Predicate
                  , HasDefault
                >*
              , ArgPack*
            );

        // Builds an overload set including satisfies functions defined
        // in base classes.
        using Next::satisfies;
#endif  // Borland workarounds needed

        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename KW, typename T2>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::arg_list<
            ::crospi_boost::parameter::aux::tagged_argument<KW,T2>
          , self
        >
            operator,(
                ::crospi_boost::parameter::aux::tagged_argument<KW,T2> const& x
            ) const
        {
            return ::crospi_boost::parameter::aux::arg_list<
                ::crospi_boost::parameter::aux::tagged_argument<KW,T2>
              , self
            >(x, *this);
        }

        // MPL sequence support
        typedef self type;        // Convenience for users
        typedef Next tail_type;   // For the benefit of iterators
        // For dispatching to sequence intrinsics
        typedef ::crospi_boost::parameter::aux::arg_list_tag tag;
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_HAS_PERFECT_FORWARDING

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename ...ArgTuples>
    struct arg_list_cons;

    template <>
    struct arg_list_cons<>
    {
        using type = ::crospi_boost::parameter::aux::empty_arg_list;
    };

    template <typename ArgTuple0, typename ...Tuples>
    struct arg_list_cons<ArgTuple0,Tuples...>
    {
        using type = ::crospi_boost::parameter::aux::arg_list<
            typename ArgTuple0::tagged_arg
          , typename ::crospi_boost::parameter::aux::arg_list_cons<Tuples...>::type
          , typename ArgTuple0::emits_errors
        >;
    };

    template <
        typename Keyword
      , typename TaggedArg
      , typename EmitsErrors = ::crospi_boost::mp11::mp_true
    >
    struct flat_like_arg_tuple
    {
        using tagged_arg = TaggedArg;
        using emits_errors = EmitsErrors;
    };

    template <typename ...ArgTuples>
    class flat_like_arg_list
      : public ::crospi_boost::parameter::aux::arg_list_cons<ArgTuples...>::type
    {
        using _base_type = typename ::crospi_boost::parameter::aux
        ::arg_list_cons<ArgTuples...>::type;

     public:
        inline CROSPI_BOOST_CONSTEXPR flat_like_arg_list(
            typename _base_type::tagged_arg const& head
          , typename _base_type::tail_type const& tail
        ) : _base_type(head, tail)
        {
        }

        template <typename ...Args>
        inline CROSPI_BOOST_CONSTEXPR flat_like_arg_list(Args&&... args)
          : _base_type(::std::forward<Args>(args)...)
        {
        }

        using _base_type::operator[];
        using _base_type::satisfies;

        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename TaggedArg>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::flat_like_arg_list<
            ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                typename TaggedArg::base_type::key_type
              , typename TaggedArg::base_type
            >
          , ArgTuples...
        >
            operator,(TaggedArg const& x) const
        {
            return ::crospi_boost::parameter::aux::flat_like_arg_list<
                ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                    typename TaggedArg::base_type::key_type
                  , typename TaggedArg::base_type
                >
              , ArgTuples...
            >(
                static_cast<typename TaggedArg::base_type const&>(x)
              , static_cast<_base_type const&>(*this)
            );
        }
    };

    template <>
    class flat_like_arg_list<>
      : public ::crospi_boost::parameter::aux::empty_arg_list
    {
        using _base_type = ::crospi_boost::parameter::aux::empty_arg_list;

     public:
        template <typename ...Args>
        inline CROSPI_BOOST_CONSTEXPR flat_like_arg_list(Args&&... args)
          : _base_type(::std::forward<Args>(args)...)
        {
        }

        using _base_type::operator[];
        using _base_type::satisfies;

        // Comma operator to compose argument list without using parameters<>.
        // Useful for argument lists with undetermined length.
        template <typename TaggedArg>
        inline CROSPI_BOOST_CONSTEXPR ::crospi_boost::parameter::aux::flat_like_arg_list<
            ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                typename TaggedArg::base_type::key_type
              , typename TaggedArg::base_type
            >
        >
            operator,(TaggedArg const& x) const
        {
            return ::crospi_boost::parameter::aux::flat_like_arg_list<
                ::crospi_boost::parameter::aux::flat_like_arg_tuple<
                    typename TaggedArg::base_type::key_type
                  , typename TaggedArg::base_type
                >
            >(
                static_cast<typename TaggedArg::base_type const&>(x)
              , static_cast<_base_type const&>(*this)
            );
        }
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

#include <crospi_boost/mpl/iterator_tags.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // MPL sequence support
    template <typename ArgumentPack>
    struct arg_list_iterator
    {
        typedef ::crospi_boost::mpl::forward_iterator_tag category;

        // The incremented iterator
        typedef ::crospi_boost::parameter::aux
        ::arg_list_iterator<typename ArgumentPack::tail_type> next;

        // dereferencing yields the key type
        typedef typename ArgumentPack::key_type type;
    };

    template <>
    struct arg_list_iterator< ::crospi_boost::parameter::aux::empty_arg_list>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/begin_end_fwd.hpp>

// MPL sequence support
namespace crospi_boost { namespace mpl {

    template <>
    struct begin_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename S>
        struct apply
        {
            typedef ::crospi_boost::parameter::aux::arg_list_iterator<S> type;
        };
    };

    template <>
    struct end_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename>
        struct apply
        {
            typedef ::crospi_boost::parameter::aux::arg_list_iterator<
                ::crospi_boost::parameter::aux::empty_arg_list
            > type;
        };
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/parameter/value_type.hpp>
#include <crospi_boost/mpl/has_key_fwd.hpp>
#include <crospi_boost/type_traits/is_void.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct has_key_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename ArgList, typename Keyword>
        struct apply
        {
            typedef typename ::crospi_boost::mpl::if_<
                ::crospi_boost::is_void<
                    typename ::crospi_boost::parameter
                    ::value_type<ArgList,Keyword,void>::type
                >
              , ::crospi_boost::mpl::false_
              , ::crospi_boost::mpl::true_
            >::type type;
        };
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/mpl/count_fwd.hpp>
#include <crospi_boost/mpl/int.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct count_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename ArgList, typename Keyword>
        struct apply
        {
            typedef typename ::crospi_boost::mpl::if_<
                ::crospi_boost::is_void<
                    typename ::crospi_boost::parameter
                    ::value_type<ArgList,Keyword,void>::type
                >
              , ::crospi_boost::mpl::int_<0>
              , ::crospi_boost::mpl::int_<1>
            >::type type;
        };
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/mpl/key_type_fwd.hpp>
#include <crospi_boost/mpl/identity.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct key_type_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename ArgList, typename Keyword>
        struct apply
        {
            typedef typename ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_void<
                    typename ::crospi_boost::parameter
                    ::value_type<ArgList,Keyword,void>::type
                >
              , void
              , ::crospi_boost::mpl::identity<Keyword>
            >::type type;
        };
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/mpl/value_type_fwd.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct value_type_impl< ::crospi_boost::parameter::aux::arg_list_tag>
      : ::crospi_boost::mpl::key_type_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/mpl/at_fwd.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct at_impl< ::crospi_boost::parameter::aux::arg_list_tag>
      : ::crospi_boost::mpl::key_type_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
    };
}} // namespace crospi_boost::mpl

#include <crospi_boost/mpl/order_fwd.hpp>
#include <crospi_boost/mpl/void.hpp>
#include <crospi_boost/mpl/find.hpp>
#include <crospi_boost/mpl/distance.hpp>

namespace crospi_boost { namespace mpl {

    template <>
    struct order_impl< ::crospi_boost::parameter::aux::arg_list_tag>
    {
        template <typename ArgList, typename Keyword>
        struct apply
        {
            typedef typename ::crospi_boost::mpl::find<ArgList,Keyword>::type Itr;
            typedef typename ::crospi_boost::mpl::eval_if<
                ::crospi_boost::is_void<
                    typename ::crospi_boost::parameter
                    ::value_type<ArgList,Keyword,void>::type
                >
              , ::crospi_boost::mpl::identity< ::crospi_boost::mpl::void_>
              , ::crospi_boost::mpl::distance<
                    Itr
                  , ::crospi_boost::parameter::aux::arg_list_iterator<
                        ::crospi_boost::parameter::aux::empty_arg_list
                    >
                >
            >::type type;
        };
    };
}} // namespace crospi_boost::mpl

#endif  // include guard

