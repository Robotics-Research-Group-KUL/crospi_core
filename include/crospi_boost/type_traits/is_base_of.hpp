
//  (C) Copyright Rani Sharoni 2003-2005.
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.
 
#ifndef CROSPI_BOOST_TT_IS_BASE_OF_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_BASE_OF_HPP_INCLUDED

#include <crospi_boost/type_traits/is_base_and_derived.hpp>
#include <crospi_boost/type_traits/is_same.hpp>
#include <crospi_boost/type_traits/is_class.hpp>

namespace crospi_boost {

   namespace detail{
      template <class B, class D>
      struct is_base_of_imp
      {
          typedef typename remove_cv<B>::type ncvB;
          typedef typename remove_cv<D>::type ncvD;
          CROSPI_BOOST_STATIC_CONSTANT(bool, value = (
            (::crospi_boost::detail::is_base_and_derived_impl<ncvB,ncvD>::value) ||
            (::crospi_boost::is_same<ncvB,ncvD>::value && ::crospi_boost::is_class<ncvB>::value)));
      };
   }

   template <class Base, class Derived> struct is_base_of
      : public integral_constant<bool, (::crospi_boost::detail::is_base_of_imp<Base, Derived>::value)> {};

   template <class Base, class Derived> struct is_base_of<Base, Derived&> : false_type{};
   template <class Base, class Derived> struct is_base_of<Base&, Derived&> : false_type{};
   template <class Base, class Derived> struct is_base_of<Base&, Derived> : false_type{};

} // namespace crospi_boost

#endif // CROSPI_BOOST_TT_IS_BASE_AND_DERIVED_HPP_INCLUDED
