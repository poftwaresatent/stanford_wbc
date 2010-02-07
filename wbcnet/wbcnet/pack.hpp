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

/** \file pack.hpp Core serialization mechanisms. */

#ifndef WBCNET_PACK_HPP
#define WBCNET_PACK_HPP

#include <wbcnet/endian_mode.hpp>
#include <vector>
#include <string>


namespace wbcnet {
  
  
  /**
     Packable fields can be easily sent over some communication
     channel. In order to keep serialization independent from the
     actual IPC mechanism, the Pack() and Unpack() methods work with
     "raw" pointers into some buffer memory.
     
     Conceptually, any Packable thing is a continuous block of memory
     starting at GetData() that can be subdivided into a number
     GetItemCount() of identically GetItemSize() sized items. The size
     of each item can be used to perform endian-translations, and the
     item count allows to know how many bytes in total have to be
     serialized.
  */
  class Packable
  {
  public:
    Packable(/** Well, kinda overkill, but useful for debugging. */
	     std::string const & name,
	     /** Instead of choosing a general but heavyweight
		 smart-pointer solution, users can set this flag to
		 true if they are passing a Packable instance to
		 Packer::AddFixedField() or Packer::AddVariableField()
		 and want the Packer to delete this object after
		 use. This is typically the case, so probably you'll
		 want to say "true" here. */
	     bool packer_owned,
	     /** When packable things have a fixed size, Packer is
		 able to cache some things. However, we want to stay
		 flexible with regards to resizing payloads, so you
		 can say "false" here if you need that flexibility
		 (and are willing to accept the runtime overhead each
		 time we need to figure out a message size). */
	     bool fixed_size);
    virtual ~Packable();
    
    /**
       Subclasses have to tell the base class the start address of
       where they reside in memory.
    */
    virtual char * GetData() = 0;
    
    char const * GetConstData() const
    { return const_cast<Packable*>(this)->GetData(); }
    
    /**
       Subclasses have to tell the base class how big each contained
       item is. For example, if the subclass wraps an array of twelve
       doubles, then GetItemSize() must return sizeof(double).
    */
    virtual int GetItemSize() const = 0;
    
    /**
       Subclasses have to tell the base class how many identically
       sized items they represent. For example, if the subclass wraps
       an array of twelve doubles, then GetItemCount() must return 12.
    */
    virtual int GetItemCount() const = 0;
    
    /**
       Serialize the Packable into the memory area starting at the
       buffer pointer. This is done blindly, so you'd better make sure
       there's enough room before calling this method. Typically, you
       let Proxy::DoPack() handle those checks.
       
       \return The address immediately following the packed data,
       i.e. <code>buffer + GetItemSize() * GetItemCount()</code>.
    */
    char * Pack(char * buffer, endian_mode_t endian_mode) const;
    
    /**
       Deserialize the Packable item from the memory area starting at
       the buffer pointer. Like Pack(), no checks are performed, but
       usually this ends up being called by Proxy::CheckID(),
       Proxy::DoUnpackHeader(), or Proxy::DoUnpackPayload(), which
       perform such checks.
       
       \return The address immediately following the packed data,
       i.e. <code>buffer + GetItemSize() * GetItemCount()</code>.       
    */
    char const * Unpack(char const * buffer, endian_mode_t endian_mode);
    
    std::string const name;
    
    /**
       If this is true, then the Packer destructor will delete this
       instance for you.
    */
    bool const packer_owned;
    
    /**
       If this is true, then the Packer will not cache size-related
       things for this instance.
    */
    bool const fixed_size;
  };
  
  
  /**
     Collection of Packable things. Proxy relies heavily on Packer
     instances: the message ID is stored in in a Packer with only one
     fixed field, the message header is represented by a Packer that
     has only fixed-sized fields, and the payload is a Packer with any
     combination of fixed and variable fields.
  */
  class Packer
  {
  public:
    Packer();
    virtual ~Packer();
    
    /**
       Add a Packable to the list of fields that should be serialized
       and deserialized by this Packer.
    */
    void AddField(Packable * field);
    
    /**
       \return The sum of all field's Packable::GetItemSize() *
       Packable::GetItemCount(). If all fields are
       Packable::fixed_size, this is a simple lookup.
    */
    int GetPackSize() const;
    
    /**
       Iteratively call Packable::Pack() on all registered fields. No
       sanity checks are performed, the caller has to make sure that
       there is enough memory available at the buffer
       pointer. Usually, you let Proxy::DoPack() handle this for you.
       
       \return A pointer one-past the last written byte, so you can
       chain it with PackAll() of other Packer instances.
    */
    char * PackAll(char * buffer, endian_mode_t endian_mode) const;
    
    /**
       Iteratively call Packable::Unpack() on all registered
       fields. No sanity checks are performed, the caller has to make
       sure that there is enough memory available at the buffer
       pointer. Usually, you let Proxy::DoUnpackHeader() or
       Proxy::DoUnpackPayload() handle this for you.
       
       \return A pointer one-past the last unpacked byte, so you can
       chain it with UnpackAll() of other Packer instances.
    */
    char const * UnpackAll(char const * buffer, endian_mode_t endian_mode);
    
    /** \return buffer + GetPackSize() */
    char * Skip(char * buffer) const;
    
    /** Same as Skip(), but with const pointers */
    char const * SkipConst(char const * buffer) const
    { return Skip(const_cast<char*>(buffer)); }
    
    bool IsEmpty() const
    { return m_field.empty() && m_variable_field.empty(); }
    
  protected:
    typedef std::vector<Packable*> field_table_t;
    
    field_table_t m_field;
    field_table_t m_variable_field;
    int m_fixed_pack_size;
  };
  
  
  /** Trait for 2nd param of class ValuePack<>. */
  struct fixed_count {
    static bool fixed_size() { return true; }
    typedef int const count_t;
  };
    
  /** Trait for 2nd param of class ValuePack<>. */
  struct variable_count {
    static bool fixed_size() { return false; }
    typedef int count_t;
  };
  
  /**
     Generic way of making a fundamental data type Packable. Takes a
     pointer to an existing memory location and the number of items
     residing there. Thus you can wrap a fixed sized array. Not
     recommended for non-fundamental types, because then it might
     break because of compiler differences or endian-conversions.
  */
  template<typename value_t, typename count_trait>
  class ValuePack
    : public Packable
  {
  public:
    ValuePack(std::string const & name, bool packer_owned, value_t * _data, int _count)
      : Packable(name, packer_owned, count_trait::fixed_size()),
	data(_data), count(_count) {}
    
    virtual char * GetData()         { return (char*) data; }
    virtual int GetItemSize()  const { return sizeof(value_t); }
    virtual int GetItemCount() const { return count; }
    
    value_t * data;
    typename count_trait::count_t count;
  };
  
  
  class VectorStorageAPI;
  
  /**
     Wrapper to make a VectorAPI instance packable. Otherwise very
     similar to ValuePack<>.
  */
  class VectorPack
    : public Packable
  {
  public:
    VectorPack(std::string const & name, bool packer_owned, bool fixed_size,
	       VectorStorageAPI * vec);
    
    virtual char * GetData();
    virtual int GetItemSize() const;
    virtual int GetItemCount() const;
    
    wbcnet::VectorStorageAPI * vec;
  };
  
  
  class MatrixStorageAPI;
  
  /**
     Wrapper to make a MatrixAPI instance packable. Otherwise very
     similar to ValuePack<>.
  */
  class MatrixPack
    : public Packable
  {
  public:
    MatrixPack(std::string const & name, bool packer_owned, bool fixed_size,
	       MatrixStorageAPI ** mx);
    
    virtual char * GetData();
    virtual int GetItemSize() const;
    virtual int GetItemCount() const;
    
    // double-indirection because we want to be able to change the
    // matrix we're wrapping
    wbcnet::MatrixStorageAPI ** mx;
  };
  
  
  struct timestamp;
  
  /**
     Makes a timestamp packable.
  */
  class TimestampPack
    : public Packable
  {
  public:
    TimestampPack(std::string const & name, bool packer_owned, timestamp * tstampPtr);
    
    virtual char * GetData();
    virtual int GetItemSize() const;
    virtual int GetItemCount() const;
    
    timestamp * tstampPtr;
  };
  
}

#endif // WBCNET_PACK_HPP
