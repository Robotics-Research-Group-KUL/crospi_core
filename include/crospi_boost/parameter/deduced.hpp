// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_DEDUCED_HPP
#define CROSPI_BOOST_PARAMETER_DEDUCED_HPP

#include <crospi_boost/parameter/aux_/use_default.hpp>

namespace crospi_boost { namespace parameter {

    // This metafunction can be used to describe the treatment of particular
    // named parameters for the purposes of overload elimination with SFINAE,
    // by placing specializations in the parameters<...> list.
    //
    // If a keyword k is specified with deduced<...>, that keyword
    // will be automatically deduced from the argument list.
    template <typename Tag>
    struct deduced
    {
        typedef Tag key_type;
    };
}}

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/integral.hpp>
#else
#include <crospi_boost/mpl/bool.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename T>
    struct is_deduced_aux
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_false
#else
      : ::crospi_boost::mpl::false_
#endif
    {
    };

    template <typename Tag>
    struct is_deduced_aux< ::crospi_boost::parameter::deduced<Tag> >
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
      : ::crospi_boost::mp11::mp_true
#else
      : ::crospi_boost::mpl::true_
#endif
    {
    };

    template <typename T>
    struct is_deduced0
      : ::crospi_boost::parameter::aux::is_deduced_aux<typename T::key_type>::type
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/parameter/required.hpp>
#include <crospi_boost/parameter/optional.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/if.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    //
    // tag_type, has_default, and predicate --
    //
    // These metafunctions accept a ParameterSpec and extract the
    // keyword tag, whether or not a default is supplied for the
    // parameter, and the predicate that the corresponding actual
    // argument type is required match.
    //
    // a ParameterSpec is a specialization of either keyword<...>,
    // required<...>, optional<...>
    //

    template <typename T>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using has_default = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::parameter::aux::is_required<T>
      , ::crospi_boost::mp11::mp_false
      , ::crospi_boost::mp11::mp_true
    >;
#else
    struct has_default
      : ::crospi_boost::mpl::if_<
            ::crospi_boost::parameter::aux::is_required<T>
          , ::crospi_boost::mpl::false_
          , ::crospi_boost::mpl::true_
        >::type
    {
    };
#endif

    template <typename T>
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
    using is_deduced = ::crospi_boost::mp11::mp_if<
        ::crospi_boost::mp11::mp_if<
            ::crospi_boost::parameter::aux::is_optional<T>
          , ::crospi_boost::mp11::mp_true
          , ::crospi_boost::parameter::aux::is_required<T>
        >
      , ::crospi_boost::parameter::aux::is_deduced0<T>
      , ::crospi_boost::mp11::mp_false
    >;
#else
    struct is_deduced
      : ::crospi_boost::mpl::if_<
            typename ::crospi_boost::mpl::if_<
                ::crospi_boost::parameter::aux::is_optional<T>
              , ::crospi_boost::mpl::true_
              , ::crospi_boost::parameter::aux::is_required<T>
            >::type
          , ::crospi_boost::parameter::aux::is_deduced0<T>
          , ::crospi_boost::mpl::false_
        >::type
    {
    };
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

