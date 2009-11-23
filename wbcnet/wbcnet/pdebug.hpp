/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
   \file pdebug.hpp Macros for optional debug messages.
   
   You can enable these messages by defining the preprocessor symbol
   WBCNET_DEBUG or WBCNET_VERBOSE_DEBUG.
*/

#ifndef WBCNET_PDEBUG_HPP
#define WBCNET_PDEBUG_HPP


#ifdef WIN32
# undef CPP_UNDERSTANDS_VARARG
#else
# define CPP_UNDERSTANDS_VARARG
#endif


#ifdef CPP_UNDERSTANDS_VARARG
# define PDEBUG_OUT(fmt, arg...) fprintf(stderr, "%s(): " fmt, __func__, ## arg)
# define PDEBUG_OFF(fmt, arg...)
#else
inline void PDEBUG_OUT(char const * fmt, ...) {}
inline void PDEBUG_OFF(char const * fmt, ...) {}
#endif


#ifdef WBCNET_VERBOSE_DEBUG
# define WBCNET_DEBUG
# define PVDEBUG PDEBUG_OUT
#else
# define PVDEBUG PDEBUG_OFF
#endif

#ifdef WBCNET_DEBUG
# include <stdio.h>
# define PDEBUG PDEBUG_OUT
#else
# define PDEBUG PDEBUG_OFF
#endif


#endif // WBCNET_PDEBUG_HPP
