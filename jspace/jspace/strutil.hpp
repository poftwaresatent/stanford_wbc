/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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
   \file wbcnet/strutil.hpp
   \author Roland Philippsen (roland DOT philippsen AT gmx DOT net)
*/


#ifndef SFL_STRUTIL_HPP
#define SFL_STRUTIL_HPP


#include <string>
#include <sstream>


namespace wbcnet {
  
  template<typename displayable_t, typename prefix_t>
  std::string displayString(displayable_t const & displayable, prefix_t const & prefix)
  {
    std::ostringstream result;
    displayable.display(result, prefix);
    return result.str();
  }
  
}


namespace sfl {
  
  /** convert "anything" to a string by using its output operator */
  template<typename Foo>
  std::string to_string(const Foo & foo) {
    std::ostringstream os;
    os << foo;
    return os.str();
  }
  
  /** booleans are better represented by "true" and "false" than by 1 and 0. */
  template<>
  std::string to_string<bool>(const bool & flag);
  
  /** convert a string to "something" based on its input operator */
  template<typename Foo>
  bool string_to(const std::string & str, Foo & foo) {
    Foo bar;
    std::istringstream is(str);
    if( ! (is >> bar))
      return false;
    foo = bar;
    return true;
  }
  
  /** booleans are converted by string matching: "true", "True",
      "TRUE", "on", "On", or "ON" yield a true boolean, whereas
      "false", "False", "FALSE", "off", "Off", or "OFF" yield a false
      boolean. Anything else results in a failure (and foo is not
      touched). */
  template<>
  bool string_to<bool>(const std::string & str, bool & foo);
  
  /** very useful for booleans that are encoded as char, int, short,
      ... sets them to 1 if string_to<bool> yields true, or to 0 if
      string_to<bool> yields false, but doesn't touch foo if
      string_to<bool> failed. */
  template<typename Foo>
  bool string_to_bool(const std::string & str, Foo & foo) {
    bool bar;
    if( ! string_to(str, bar))
      return false;
    foo = bar ? 1 : 0;
    return true;
  }

  
  /**
     Split a string at the first occurrence of a separator, and store
     the two portions in head and tail. For example,
     "head:tail_or:whatever::comes:then" with separator ':' will yield
     "head" and "tail_or:whatever::comes:then". The same string split
     along '_' would yield "head:tail" and "or:whatever::comes:then".
     
     Note that it's OK to pass the same instance as input and tail,
     but DO NOT pass the head twice.
     
     \code
     for (int ii(1); ii < argc; ++ii) {
       string head;
       string tail(argv[ii]);
       while (splitstring(tail, ':', head, tail))
         cout << head << "\n";
       cout << head << "\n";
     }
     \endcode
     
     \return true if there is more to be extracted, allowing you to
     easily tokenize a string. But see also tokenize() which does just
     that.
  */
  bool splitstring(std::string const & input, char separator,
		   std::string & head, std::string & tail);
  
  
  /**
     For any tokenlist_t that accepts push_back(string const &) and
     can return its size().
  */
  template<typename tokenlist_t>
  size_t tokenize(std::string const & input, char separator, tokenlist_t & output) {
    std::string head;
    std::string tail(input);
    while (splitstring(tail, separator, head, tail))
      output.push_back(head);
    output.push_back(head);
    return output.size();
  }
  
  
  /**
     For any tokenlist_t whose operator[]() returns a string const &
     and which can return its size().
  */
  template<typename tokenlist_t, typename value_t>
  bool token_to(tokenlist_t const & tokenlist, size_t index, value_t & value) {
    if (index >= tokenlist.size())
      return false;
    return string_to(tokenlist[index], value);
  }
  
}

#endif // SFL_STRUTIL_HPP
