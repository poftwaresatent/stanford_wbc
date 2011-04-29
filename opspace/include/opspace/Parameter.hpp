/*
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 *
 * Author: Roland Philippsen
 *         http://cs.stanford.edu/group/manips/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef OPSPACE_PARAMETER_HPP
#define OPSPACE_PARAMETER_HPP

#include <jspace/Status.hpp>
#include <jspace/wrap_eigen.hpp>
#include <boost/shared_ptr.hpp>
#include <map>


namespace opspace {
  
  
  using jspace::Status;
  using jspace::Vector;
  using jspace::Matrix;
  
  
  /**
     Enumeration type for task parameter types.
  */
  typedef enum {
    PARAMETER_TYPE_VOID,	//!< no data (e.g. invalid type code)
    PARAMETER_TYPE_STRING,	//!< mapped to std::string
    PARAMETER_TYPE_INTEGER,	//!< mapped to int
    PARAMETER_TYPE_REAL,	//!< mapped to double
    PARAMETER_TYPE_VECTOR,	//!< mapped to jspace::Vector
    PARAMETER_TYPE_MATRIX	//!< mapped to jspace::Matrix
  } parameter_type_t;


  typedef enum {
    PARAMETER_FLAG_DEFAULT = 0,
    PARAMETER_FLAG_NOLOG = 1,
    PARAMETER_FLAG_READONLY = 2
  } parameter_flags_t;
  
  
  class ParameterReflection;
  
  
  /**
     Abstract base for all (task) parameters. Fairly minimal for now:
     parameters have a name, a type, and (optionally) an associated
     checker. Ideas for future extensions are e.g. documentation
     strings, optional bounds for automatic checks, and a more generic
     type interface (instead of relying on parameter_type_t).
     
     \note This base class can be instantiated, but it just behaves
     like a PARAMETER_TYPE_VOID parameter: you cannot get or set
     anything.
  */
  class Parameter
  {
  public:
    std::string const name_;
    parameter_type_t const type_;
    parameter_flags_t const flags_;
    ParameterReflection const * checker_;
    
    Parameter(std::string const & name,
	      parameter_type_t type,
	      parameter_flags_t flags,
	      ParameterReflection const * checker);
    
    virtual ~Parameter();
    
    virtual int const * getInteger() const;
    virtual std::string const * getString() const;
    virtual double const * getReal() const;
    virtual Vector const * getVector() const;
    virtual Matrix const * getMatrix() const;
    
    virtual Status set(int value);
    virtual Status set(std::string const & value);
    virtual Status set(double value);
    virtual Status set(Vector const & value);
    virtual Status set(Matrix const & value);
    
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  };
  
  
  /** Implementation for integer parameters: a single int value. */
  class IntegerParameter : public Parameter {
  public:
    IntegerParameter(std::string const & name,
		     parameter_flags_t flags,
		     ParameterReflection const * checker,
		     int * instance);
    virtual int const * getInteger() const { return integer_; }
    virtual Status set(int integer);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    int * integer_;
  };
  
  
  /** Implementation for string parameters: a single std::string value. */
  class StringParameter : public Parameter {
  public:
    StringParameter(std::string const & name,
		    parameter_flags_t flags,
		    ParameterReflection const * checker,
		    std::string * instance);
    virtual std::string const * getString() const { return string_; }
    virtual Status set(std::string const & value);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    std::string * string_;
  };
  
  
  /** Implementation for real parameters: a single double value. */
  class RealParameter : public Parameter {
  public:
    RealParameter(std::string const & name,
		  parameter_flags_t flags,
		  ParameterReflection const * checker,
		  double * real);
    virtual double const * getReal() const { return real_; }
    virtual Status set(double real);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    double * real_;
  };
  
  
  /** Implementation for vector parameters: a vector of double values. */
  class VectorParameter : public Parameter {
  public:
    VectorParameter(std::string const & name,
		    parameter_flags_t flags,
		    ParameterReflection const * checker,
		    Vector * vector);
    virtual Vector const * getVector() const { return vector_; }
    virtual Status set(Vector const & vector);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Vector * vector_;
  };
  
  
  /** Implementation for matrix parameters: a matrix of double values. */
  class MatrixParameter : public Parameter {
  public:
    MatrixParameter(std::string const & name,
		    parameter_flags_t flags,
		    ParameterReflection const * checker,
		    Matrix * matrix);
    virtual Matrix const * getMatrix() const { return matrix_; }
    virtual Status set(Matrix const & matrix);
    virtual void dump(std::ostream & os, std::string const & prefix) const;
  protected:
    Matrix * matrix_;
  };
  
  
  typedef std::map<std::string, Parameter *> parameter_lookup_t;
  
  
  /**
     Base for classes that reflect (some of) their parameters. Manages
     a table of Parameter instances and provides default
     implementations for parameter checker methods.
  */
  class ParameterReflection
  {
  public:
    ParameterReflection(std::string const & type_name,
			std::string const & instance_name);
    
    virtual ~ParameterReflection();
    
    /** Default implementation always returns succes. */
    virtual Status check(int const * param, int value) const;

    /** Default implementation always returns succes. */
    virtual Status check(std::string const * param, std::string const & value) const;
    
    /** Default implementation always returns succes. */
    virtual Status check(double const * param, double value) const;
    
    /** Default implementation always returns succes. */
    virtual Status check(Vector const * param, Vector const & value) const;
    
    /** Default implementation always returns succes. */
    virtual Status check(Matrix const * param, Matrix const & value) const;
    
    inline std::string const & getName() const { return instance_name_; }
    inline std::string const & getTypeName() const { return type_name_; }
    
    /**
       \return A pointer to the Parameter subclass instance which
       represents a certain named parameter, or zero if the name does
       not match. You can use getParameterTable() to inspect the list
       of parameters.
    */
    Parameter * lookupParameter(std::string const & name);
    
    /**
       \return A const pointer to the Parameter subclass instance
       which represents a certain named parameter, or zero if the name
       does not match. You can use getParameterTable() to inspect the
       list of parameters.
    */
    Parameter const * lookupParameter(std::string const & name) const;
    
    /**
       \return A pointer to the Parameter subclass instance which
       represents a certain named parameter AND matches the given
       type, or zero if the name or the type does not match. You can
       use getParameterTable() to inspect the list of parameters.
    */
    Parameter * lookupParameter(std::string const & name, parameter_type_t type);

    /**
       \return A const pointer to the Parameter subclass instance
       which represents a certain named parameter AND matches the
       given type, or zero if the name or the type does not match. You
       can use getParameterTable() to inspect the list of parameters.
    */
    Parameter const * lookupParameter(std::string const & name, parameter_type_t type) const;
    
    parameter_lookup_t const & getParameterTable() const { return parameter_lookup_; }
    
    virtual void dump(std::ostream & os,
		      std::string const & title,
		      std::string const & prefix) const;
    
  protected:
    std::string const type_name_;
    std::string const instance_name_;
    
    /**
       Used by subclasse to make one of their fields accessible to the
       outside. The parameter then becomes available through the
       lookupParameter() and getParameterTable() methods.
       
       \note Everyone is granted read access to the parameter. Write
       access is protected in two ways: the caller needs to have a
       non-const handle on the Task instance, and the Parameter::set()
       method will call Task::check() before actually writing a new
       value into the pointer provided here at declaration time.
    */
    IntegerParameter * declareParameter(std::string const & name,
					int * integer,
					parameter_flags_t flags = PARAMETER_FLAG_DEFAULT);
    
    /** See also declareParameter(std::string const &, int *)... */
    StringParameter * declareParameter(std::string const & name,
				       std::string * instance,
				       parameter_flags_t flags = PARAMETER_FLAG_DEFAULT);
    
    /** See also declareParameter(std::string const &, int *)... */
    RealParameter * declareParameter(std::string const & name,
				     double * real,
				     parameter_flags_t flags = PARAMETER_FLAG_DEFAULT);

    /** See also declareParameter(std::string const &, int *)... */
    VectorParameter * declareParameter(std::string const & name,
				       Vector * vector,
				       parameter_flags_t flags = PARAMETER_FLAG_DEFAULT);

    /** See also declareParameter(std::string const &, int *)... */
    MatrixParameter * declareParameter(std::string const & name,
				       Matrix * matrix,
				       parameter_flags_t flags = PARAMETER_FLAG_DEFAULT);
    
  private:
    parameter_lookup_t parameter_lookup_;
  };
  
  
  class ReflectionRegistry
  {
  public:
    struct enumeration_entry_s {
      std::string type_name, instance_name, parameter_name;
      Parameter * parameter;
    };
    typedef std::vector<enumeration_entry_s> enumeration_t;
    
    
    void add(boost::shared_ptr<ParameterReflection> instance);
    
    boost::shared_ptr<ParameterReflection> find(std::string const & type_name,
						std::string const & instance_name);
    
    void enumerate(enumeration_t & enumeration);

    Parameter * lookupParameter(std::string const & type_name,
				std::string const & instance_name,
				std::string const & parameter_name);
    
    Parameter const * lookupParameter(std::string const & type_name,
				      std::string const & instance_name,
				      std::string const & parameter_name) const;
    
    Parameter * lookupParameter(std::string const & type_name,
				std::string const & instance_name,
				std::string const & parameter_name,
				parameter_type_t parameter_type);
    
    Parameter const * lookupParameter(std::string const & type_name,
				      std::string const & instance_name,
				      std::string const & parameter_name,
				      parameter_type_t parameter_type) const;
    
  private:
    typedef std::map<std::string, boost::shared_ptr<ParameterReflection> > instance_map_t;
    typedef std::map<std::string, instance_map_t> type_map_t;
    type_map_t type_map_;
  };
  
  
  class ParameterLog
  {
  public:
    template<typename parameter_t, typename storage_t>
    struct log_s {
      explicit log_s(parameter_t const * pp): parameter(pp) {}
      parameter_t const * parameter;
      std::vector<storage_t> log;
    };
    
    ParameterLog(std::string const & name, parameter_lookup_t const & parameter_lookup);
    
    void update(long long timestamp);
    void writeFiles(std::string const & prefix, std::ostream * progress) const;
    
    std::string const name;
    std::vector<long long> timestamp;
    std::vector<log_s<IntegerParameter, int> > intlog;
    std::vector<log_s<StringParameter, std::string> > strlog;
    std::vector<log_s<RealParameter, double> > reallog;
    std::vector<log_s<VectorParameter, Vector> > veclog;
    std::vector<log_s<MatrixParameter, Matrix> > mxlog;
  };
  
}

#endif // OPSPACE_PARAMETER_HPP
