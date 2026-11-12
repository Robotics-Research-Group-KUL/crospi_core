// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_MAKE_PARAMETER_SPEC_ITEMS_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_MAKE_PARAMETER_SPEC_ITEMS_HPP

namespace crospi_boost { namespace parameter { namespace aux {

    // This recursive metafunction forwards successive elements of
    // parameters::parameter_spec to make_deduced_items<>.
    // -- Cromwell D. Enage
    template <typename SpecSeq>
    struct make_deduced_list;

    // Helper for match_parameters_base_cond<...>, below.
    template <typename ArgumentPackAndError, typename SpecSeq>
    struct match_parameters_base_cond_helper;

    // Helper metafunction for make_parameter_spec_items<...>, below.
    template <typename SpecSeq, typename ...Args>
    struct make_parameter_spec_items_helper;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/void.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename SpecSeq>
    struct make_parameter_spec_items_helper<SpecSeq>
    {
        typedef ::crospi_boost::parameter::void_ type;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/pack/make_deduced_items.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/list.hpp>
#else
#include <crospi_boost/mpl/front.hpp>
#include <crospi_boost/mpl/pop_front.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename SpecSeq>
    struct make_deduced_list_not_empty
      : ::crospi_boost::parameter::aux::make_deduced_items<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            ::crospi_boost::mp11::mp_front<SpecSeq>
#else
            typename ::crospi_boost::mpl::front<SpecSeq>::type
#endif
          , ::crospi_boost::parameter::aux::make_deduced_list<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_pop_front<SpecSeq>
#else
                typename ::crospi_boost::mpl::pop_front<SpecSeq>::type
#endif
            >
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/eval_if.hpp>
#include <crospi_boost/mpl/empty.hpp>
#include <crospi_boost/mpl/identity.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename SpecSeq>
    struct make_deduced_list
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_if<
            ::crospi_boost::mp11::mp_empty<SpecSeq>
          , ::crospi_boost::mp11::mp_identity< ::crospi_boost::parameter::void_>
#else
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::mpl::empty<SpecSeq>
          , ::crospi_boost::mpl::identity< ::crospi_boost::parameter::void_>
#endif
          , ::crospi_boost::parameter::aux::make_deduced_list_not_empty<SpecSeq>
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <type_traits>
#else
#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/pair.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/type_traits/is_same.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename ArgumentPackAndError>
    struct is_arg_pack_error_void
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::is_same<
                typename ::crospi_boost::mpl::second<ArgumentPackAndError>::type
              , ::crospi_boost::parameter::void_
            >
          , ::crospi_boost::mpl::true_
          , ::crospi_boost::mpl::false_
        >::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11

namespace crospi_boost { namespace parameter { namespace aux {

    // Checks if the arguments match the criteria of overload resolution.
    // If NamedList satisfies the PS0, PS1, ..., this is a metafunction
    // returning parameters.  Otherwise it has no nested ::type.
    template <typename ArgumentPackAndError, typename SpecSeq>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using match_parameters_base_cond = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_empty<SpecSeq>
      , ::std::is_same<
            ::crospi_boost::mp11::mp_at_c<ArgumentPackAndError,1>
          , ::crospi_boost::parameter::void_
        >
      , ::crospi_boost::parameter::aux::match_parameters_base_cond_helper<
            ArgumentPackAndError
          , SpecSeq
        >
    >;
#else
    struct match_parameters_base_cond
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::mpl::empty<SpecSeq>
          , ::crospi_boost::parameter::aux
            ::is_arg_pack_error_void<ArgumentPackAndError>
          , ::crospi_boost::parameter::aux::match_parameters_base_cond_helper<
                ArgumentPackAndError
              , SpecSeq
            >
        >
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/pack/satisfies.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename ArgumentPackAndError, typename SpecSeq>
    struct match_parameters_base_cond_helper
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_if<
#else
      : ::crospi_boost::mpl::eval_if<
#endif
            ::crospi_boost::parameter::aux::satisfies_requirements_of<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_at_c<ArgumentPackAndError,0>
              , ::crospi_boost::mp11::mp_front<SpecSeq>
#else
                typename ::crospi_boost::mpl::first<ArgumentPackAndError>::type
              , typename ::crospi_boost::mpl::front<SpecSeq>::type
#endif
            >
          , ::crospi_boost::parameter::aux::match_parameters_base_cond<
                ArgumentPackAndError
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
              , ::crospi_boost::mp11::mp_pop_front<SpecSeq>
#else
              , typename ::crospi_boost::mpl::pop_front<SpecSeq>::type
#endif
            >
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
          , ::crospi_boost::mp11::mp_false
#else
          , ::crospi_boost::mpl::false_
#endif
        >
    {
    };

    // This parameters item chaining metafunction class does not require
    // the lengths of the SpecSeq and of Args parameter pack to match.
    // Used by argument_pack to build the items in the resulting arg_list.
    // -- Cromwell D. Enage
    template <typename SpecSeq, typename ...Args>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using make_parameter_spec_items = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_empty<SpecSeq>
      , ::crospi_boost::mp11::mp_identity< ::crospi_boost::parameter::void_>
      , ::crospi_boost::parameter::aux
        ::make_parameter_spec_items_helper<SpecSeq,Args...>
    >;
#else
    struct make_parameter_spec_items
      : ::crospi_boost::mpl::eval_if<
            ::crospi_boost::mpl::empty<SpecSeq>
          , ::crospi_boost::mpl::identity< ::crospi_boost::parameter::void_>
          , ::crospi_boost::parameter::aux
            ::make_parameter_spec_items_helper<SpecSeq,Args...>
        >
    {
    };
#endif
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/aux_/pack/make_items.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename SpecSeq, typename A0, typename ...Args>
    struct make_parameter_spec_items_helper<SpecSeq,A0,Args...>
      : ::crospi_boost::parameter::aux::make_items<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
            ::crospi_boost::mp11::mp_front<SpecSeq>
#else
            typename ::crospi_boost::mpl::front<SpecSeq>::type
#endif
          , A0
          , ::crospi_boost::parameter::aux::make_parameter_spec_items<
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
                ::crospi_boost::mp11::mp_pop_front<SpecSeq>
#else
                typename ::crospi_boost::mpl::pop_front<SpecSeq>::type
#endif
              , Args...
            >
        >
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

