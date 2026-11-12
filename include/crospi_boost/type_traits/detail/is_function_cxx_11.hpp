
//  Copyright 2000 John Maddock (john@johnmaddock.co.uk)
//  Copyright 2002 Aleksey Gurtovoy (agurtovoy@meta-comm.com)
//
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

#ifndef CROSPI_BOOST_TT_IS_FUNCTION_CXX_11_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_FUNCTION_CXX_11_HPP_INCLUDED

#include <crospi_boost/type_traits/integral_constant.hpp>

namespace crospi_boost {

   template <class T>
   struct is_function : public false_type {};

#if defined(__cpp_noexcept_function_type) && !defined(CROSPI_BOOST_TT_NO_DEDUCED_NOEXCEPT_PARAM)
#define CROSPI_BOOST_TT_NOEXCEPT_PARAM , bool NE
#define CROSPI_BOOST_TT_NOEXCEPT_DECL noexcept(NE)
#else
#define CROSPI_BOOST_TT_NOEXCEPT_PARAM
#define CROSPI_BOOST_TT_NOEXCEPT_DECL
#endif

#ifdef _MSC_VER
#define CROSPI_BOOST_TT_DEF_CALL __cdecl
#else
#define CROSPI_BOOST_TT_DEF_CALL
#endif
   
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // Reference qualified:

#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // rvalue reference qualified:

#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)&& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)&& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};


#if defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)

#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_X64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
   
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // rvalue reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)&&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)&&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)&&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

#endif // _MSC_VER

   // All over again for msvc with noexcept:

#if defined(CROSPI_BOOST_TT_NO_DEDUCED_NOEXCEPT_PARAM) && !defined(CROSPI_BOOST_TT_NO_NOEXCEPT_SEPARATE_TYPE)

#undef CROSPI_BOOST_TT_NOEXCEPT_DECL
#define CROSPI_BOOST_TT_NOEXCEPT_DECL noexcept

#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // Reference qualified:

#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // rvalue reference qualified:

#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
#if !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret CROSPI_BOOST_TT_DEF_CALL(Args...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
   template <class Ret, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret(Args..., ...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};


#if defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)

#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // rvalue reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __stdcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __clrcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __fastcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_function<Ret __vectorcall(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

#endif // defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)

#endif

}

#undef CROSPI_BOOST_TT_NOEXCEPT_DECL
#undef CROSPI_BOOST_TT_NOEXCEPT_PARAM
#undef CROSPI_BOOST_TT_DEF_CALL

#endif // CROSPI_BOOST_TT_IS_FUNCTION_CXX_11_HPP_INCLUDED

