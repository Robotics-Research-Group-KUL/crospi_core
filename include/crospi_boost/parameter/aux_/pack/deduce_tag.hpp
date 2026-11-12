// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_DEDUCE_TAG_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_DEDUCE_TAG_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    template <
        typename Argument
      , typename ArgumentPack
      , typename DeducedArgs
      , typename UsedArgs
      , typename TagFn
      , typename EmitsErrors
    >
    struct deduce_tag;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/lambda_tag.hpp>
#include <crospi_boost/parameter/config.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#include <crospi_boost/mpl/lambda.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Predicate, typename Argument, typename ArgumentPack>
    struct deduce_tag_condition_mpl
      : ::crospi_boost::mpl::apply_wrap2<
            typename ::crospi_boost::mpl::lambda<
                Predicate
              , ::crospi_boost::parameter::aux::lambda_tag
            >::type
          , Argument
          , ArgumentPack
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/has_nested_template_fn.hpp>
#include <crospi_boost/mp11/list.hpp>
#include <crospi_boost/mp11/utility.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Predicate, typename Argument, typename ArgumentPack>
    struct deduce_tag_condition_mp11
    {
        using type = ::crospi_boost::mp11::mp_apply_q<
            Predicate
          , ::crospi_boost::mp11::mp_list<Argument,ArgumentPack>
        >;
    };

    template <typename Predicate, typename Argument, typename ArgumentPack>
    using deduce_tag_condition = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::parameter::aux::has_nested_template_fn<Predicate>
      , ::crospi_boost::parameter::aux
        ::deduce_tag_condition_mp11<Predicate,Argument,ArgumentPack>
      , ::crospi_boost::parameter::aux
        ::deduce_tag_condition_mpl<Predicate,Argument,ArgumentPack>
    >;
}}} // namespace crospi_boost::parameter::aux

#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/assert.hpp>
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

#include <crospi_boost/parameter/aux_/set.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_type.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_deduced.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // Helper for deduce_tag<...>, below.
    template <
        typename Argument
      , typename ArgumentPack
      , typename DeducedArgs
      , typename UsedArgs
      , typename TagFn
      , typename EmitsErrors
    >
    class deduce_tag0
    {
        typedef typename DeducedArgs::spec _spec;

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        typedef typename ::crospi_boost::parameter::aux::deduce_tag_condition<
            typename _spec::predicate
#else
        typedef typename ::crospi_boost::mpl::apply_wrap2<
            typename ::crospi_boost::mpl::lambda<
                typename _spec::predicate
              , ::crospi_boost::parameter::aux::lambda_tag
            >::type
#endif
          , Argument
          , ArgumentPack
        >::type _condition;

#if !defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        // Deduced parameter matches several arguments.
        CROSPI_BOOST_MPL_ASSERT((
            typename ::crospi_boost::mpl::eval_if<
                typename ::crospi_boost::parameter::aux::has_key_<
                    UsedArgs
                  , typename ::crospi_boost::parameter::aux::tag_type<_spec>::type
                >::type
              , ::crospi_boost::mpl::eval_if<
                    _condition
                  , ::crospi_boost::mpl::if_<
                        EmitsErrors
                      , ::crospi_boost::mpl::false_
                      , ::crospi_boost::mpl::true_
                    >
                  , ::crospi_boost::mpl::true_
                >
              , ::crospi_boost::mpl::true_
            >::type
        ));
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

     public:
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = typename ::crospi_boost::mp11::mp_if<
#else
        typedef typename ::crospi_boost::mpl::eval_if<
#endif
            _condition
          , ::crospi_boost::parameter::aux
            ::tag_deduced<UsedArgs,_spec,Argument,TagFn>
          , ::crospi_boost::parameter::aux::deduce_tag<
                Argument
              , ArgumentPack
              , typename DeducedArgs::tail
              , UsedArgs
              , TagFn
              , EmitsErrors
            >
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        >::type;
#else
        >::type type;
#endif
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/void.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>
#else
#include <crospi_boost/mpl/pair.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // Tries to deduced a keyword tag for a given Argument.
    // Returns an mpl::pair<> consisting of the tagged_argument<>,
    // and an mpl::set<> where the new tag has been inserted.
    //
    //  Argument:     The argument type to be tagged.
    //
    //  ArgumentPack: The ArgumentPack built so far.
    //
    //  DeducedArgs:  A specialization of deduced_item<> (see below).
    //                A list containing only the deduced ParameterSpecs.
    //
    //  UsedArgs:     An mpl::set<> containing the keyword tags used so far.
    //
    //  TagFn:        A metafunction class used to tag positional or deduced
    //                arguments with a keyword tag.
    template <
        typename Argument
      , typename ArgumentPack
      , typename DeducedArgs
      , typename UsedArgs
      , typename TagFn
      , typename EmitsErrors
    >
    struct deduce_tag
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_if<
            ::std::is_same<DeducedArgs,::crospi_boost::parameter::void_>
          , ::crospi_boost::mp11::mp_identity<
                ::crospi_boost::mp11::mp_list< ::crospi_boost::parameter::void_,UsedArgs>
            >
#else
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::is_same<DeducedArgs,::crospi_boost::parameter::void_>
          , ::crospi_boost::mpl::pair< ::crospi_boost::parameter::void_,UsedArgs>
#endif
          , ::crospi_boost::parameter::aux::deduce_tag0<
                Argument
              , ArgumentPack
              , DeducedArgs
              , UsedArgs
              , TagFn
              , EmitsErrors
            >
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

