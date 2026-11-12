// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_DEDUCED_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_TAG_DEDUCED_HPP

#include <crospi_boost/parameter/aux_/set.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_type.hpp>
#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/list.hpp>
#include <crospi_boost/mp11/utility.hpp>
#else
#include <crospi_boost/mpl/pair.hpp>
#include <crospi_boost/mpl/apply_wrap.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // Tags a deduced argument Arg with the keyword tag of Spec using TagFn.
    // Returns the tagged argument and the mpl::set<> UsedArgs with the
    // tag of Spec inserted.
    template <typename UsedArgs, typename Spec, typename Arg, typename TagFn>
    struct tag_deduced
    {
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        using type = ::crospi_boost::mp11::mp_list<
            ::crospi_boost::mp11::mp_apply_q<
                TagFn
              , ::crospi_boost::mp11::mp_list<
                    typename ::crospi_boost::parameter::aux::tag_type<Spec>::type
                  , Arg
                >
            >
#else
        typedef ::crospi_boost::mpl::pair<
            typename ::crospi_boost::mpl::apply_wrap2<
                TagFn
              , typename ::crospi_boost::parameter::aux::tag_type<Spec>::type
              , Arg
            >::type
#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11
          , typename ::crospi_boost::parameter::aux::insert_<
                UsedArgs
              , typename ::crospi_boost::parameter::aux::tag_type<Spec>::type
            >::type
#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
        >;
#else
        > type;
#endif
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard

