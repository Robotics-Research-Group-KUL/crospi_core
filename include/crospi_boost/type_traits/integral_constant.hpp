//  (C) Copyright John Maddock 2015. 
//  Use, modification and distribution are subject to the 
//  Boost Software License, Version 1.0. (See accompanying file 
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_TYPE_TRAITS_INTEGRAL_CONSTANT_HPP
#define CROSPI_BOOST_TYPE_TRAITS_INTEGRAL_CONSTANT_HPP

#include <crospi_boost/config.hpp>
#include <crospi_boost/detail/workaround.hpp>

#if (CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, CROSPI_BOOST_TESTED_AT(1400)) \
   || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
   || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840)) \
   || CROSPI_BOOST_WORKAROUND(__MWERKS__, CROSPI_BOOST_TESTED_AT(0x3202)) \
   || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_INTEL_CXX_VERSION, CROSPI_BOOST_TESTED_AT(810)) )\
   || defined(CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE)


namespace crospi_boost{
   namespace mpl
   {
      template <bool B> struct bool_;
      template <class I, I val> struct integral_c;
      struct integral_c_tag;
   }
}

#else

namespace crospi_mpl_{

   template <bool B> struct bool_;
   template <class I, I val> struct integral_c;
   struct integral_c_tag;
}

namespace crospi_boost
{
   namespace mpl
   {
      using ::crospi_mpl_::bool_;
      using ::crospi_mpl_::integral_c;
      using ::crospi_mpl_::integral_c_tag;
   }
}

#endif

namespace crospi_boost{

   template <class T, T val>
   struct integral_constant
   {
      typedef mpl::integral_c_tag tag;
      typedef T value_type;
      typedef integral_constant<T, val> type;
      static const T value = val;

      operator const mpl::integral_c<T, val>& ()const
      {
         static const char data[sizeof(long)] = { 0 };
         const void* const pdata = data;
         return *static_cast<const mpl::integral_c<T, val>*>(pdata);
      }
      CROSPI_BOOST_CONSTEXPR operator T()const CROSPI_BOOST_NOEXCEPT { return val; }
      CROSPI_BOOST_CONSTEXPR T operator()()const CROSPI_BOOST_NOEXCEPT { return val; }
   };

   template <class T, T val>
   T const integral_constant<T, val>::value;
      
   template <bool val>
   struct integral_constant<bool, val>
   {
      typedef mpl::integral_c_tag tag;
      typedef bool value_type;
      typedef integral_constant<bool, val> type;
      static const bool value = val;

      operator const mpl::bool_<val>& ()const
      {
         static const char data[sizeof(long)] = { 0 };
         const void* const pdata = data;
         return *static_cast<const mpl::bool_<val>*>(pdata);
      }
      CROSPI_BOOST_CONSTEXPR operator bool()const CROSPI_BOOST_NOEXCEPT { return val; }
      CROSPI_BOOST_CONSTEXPR bool operator()()const CROSPI_BOOST_NOEXCEPT { return val; }
   };

   template <bool val>
   bool const integral_constant<bool, val>::value;

   typedef integral_constant<bool, true> true_type;
   typedef integral_constant<bool, false> false_type;

}

#endif
