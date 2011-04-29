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

#include <opspace/Parameter.hpp>
#include <fstream>

using namespace jspace;

namespace opspace {
  
  
  Parameter::
  Parameter(std::string const & name,
	    parameter_type_t type,
	    parameter_flags_t flags,
	    ParameterReflection const * checker)
    : name_(name),
      type_(type),
      flags_(flags),
      checker_(checker)
  {
    switch (type) {
    case PARAMETER_TYPE_VOID:
    case PARAMETER_TYPE_INTEGER:
    case PARAMETER_TYPE_STRING:
    case PARAMETER_TYPE_REAL:
    case PARAMETER_TYPE_VECTOR:
    case PARAMETER_TYPE_MATRIX:
      break;
    default:
      const_cast<parameter_type_t &>(type_) = PARAMETER_TYPE_VOID;
    }
  }
  
  
  Parameter::
  ~Parameter()
  {
  }
  
  
  int const * Parameter::
  getInteger() const
  {
    return 0;
  }
  
  
  std::string const * Parameter::
  getString() const
  {
    return 0;
  }


  double const * Parameter::
  getReal() const
  {
    return 0;
  }

  
  Vector const * Parameter::
  getVector() const
  {
    return 0;
  }

  
  Matrix const * Parameter::
  getMatrix() const
  {
    return 0;
  }
  
  
  Status Parameter::
  set(int value)
  {
    Status err(false, "type mismatch");
    return err;
  }
  
  
  Status Parameter::
  set(std::string const & value)
  {
    Status err(false, "type mismatch");
    return err;
  }
  
  
  Status Parameter::
  set(double value)
  {
    Status err(false, "type mismatch");
    return err;
  }
  
  
  Status Parameter::
  set(Vector const & value)
  {
    Status err(false, "type mismatch");
    return err;
  }
  
  
  Status Parameter::
  set(Matrix const & value)
  {
    Status err(false, "type mismatch");
    return err;
  }
  
  
  void Parameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : void\n";
  }
  
  
  IntegerParameter::
  IntegerParameter(std::string const & name,
		   parameter_flags_t flags,
		   ParameterReflection const * checker,
		   int * integer)
    : Parameter(name, PARAMETER_TYPE_INTEGER, flags, checker),
      integer_(integer)
  {
  }
  
  
  Status IntegerParameter::
  set(int integer)
  {
    Status st;
    if (flags_ & PARAMETER_FLAG_READONLY) {
      st.ok = false;
      st.errstr = "read-only parameter";
      return st;
    }
    if (checker_) {
      st = checker_->check(integer_, integer);
      if ( ! st) {
	return st;
      }
    }
    *integer_ = integer;
    return st;
  }
  
  
  void IntegerParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : integer = " << *integer_ << "\n";
  }
  
  
  StringParameter::
  StringParameter(std::string const & name,
		  parameter_flags_t flags,
		  ParameterReflection const * checker,
		  std::string * instance)
    : Parameter(name, PARAMETER_TYPE_STRING, flags, checker),
      string_(instance)
  {
  }
  
  
  Status StringParameter::
  set(std::string const & value)
  {
    Status st;
    if (flags_ & PARAMETER_FLAG_READONLY) {
      st.ok = false;
      st.errstr = "read-only parameter";
      return st;
    }
    if (checker_) {
      st = checker_->check(string_, value);
      if ( ! st) {
	return st;
      }
    }
    *string_ = value;
    return st;
  }
  
  
  void StringParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : string = " << *string_ << "\n";
  }


  RealParameter::
  RealParameter(std::string const & name,
		parameter_flags_t flags,
		ParameterReflection const * checker,
		double * real)
    : Parameter(name, PARAMETER_TYPE_REAL, flags, checker),
      real_(real)
  {
  }
  
  
  Status RealParameter::
  set(double real)
  {
    Status st;
    if (flags_ & PARAMETER_FLAG_READONLY) {
      st.ok = false;
      st.errstr = "read-only parameter";
      return st;
    }
    if (checker_) {
      st = checker_->check(real_, real);
      if ( ! st) {
	return st;
      }
    }
    *real_ = real;
    return st;
  }
  
  
  void RealParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : real = " << *real_ << "\n";
  }


  VectorParameter::
  VectorParameter(std::string const & name,
		  parameter_flags_t flags,
		  ParameterReflection const * checker,
		  Vector * vector)
    : Parameter(name, PARAMETER_TYPE_VECTOR, flags, checker),
      vector_(vector)
  {
  }
  
  
  Status VectorParameter::
  set(Vector const & vector)
  {
    Status st;
    if (flags_ & PARAMETER_FLAG_READONLY) {
      st.ok = false;
      st.errstr = "read-only parameter";
      return st;
    }
    if (checker_) {
      st = checker_->check(vector_, vector);
      if ( ! st) {
	return st;
      }
    }
    *vector_ = vector;
    return st;
  }
  
  
  void VectorParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : vector =\n"
       << prefix << "  " << pretty_string(*vector_) << "\n";
  }


  MatrixParameter::
  MatrixParameter(std::string const & name,
		  parameter_flags_t flags,
		  ParameterReflection const * checker,
		  Matrix * matrix)
    : Parameter(name, PARAMETER_TYPE_MATRIX, flags, checker),
      matrix_(matrix)
  {
  }
  
  
  Status MatrixParameter::
  set(Matrix const & matrix)
  {
    Status st;
    if (flags_ & PARAMETER_FLAG_READONLY) {
      st.ok = false;
      st.errstr = "read-only parameter";
      return st;
    }
    if (checker_) {
      st = checker_->check(matrix_, matrix);
      if ( ! st) {
	return st;
      }
    }
    *matrix_ = matrix;
    return st;
  }
  
  
  void MatrixParameter::
  dump(std::ostream & os, std::string const & prefix) const
  {
    os << prefix << name_ << " : matrix =\n"
       << pretty_string(*matrix_, prefix + "  ") << "\n";
  }
  
  
  ParameterReflection::
  ParameterReflection(std::string const & type_name,
		      std::string const & instance_name)
    : type_name_(type_name),
      instance_name_(instance_name)
  {
  }
  
  
  ParameterReflection::
  ~ParameterReflection()
  {
    for (parameter_lookup_t::iterator ii(parameter_lookup_.begin());
	 ii != parameter_lookup_.end(); ++ii) {
      delete ii->second;
    }
  }


  Parameter * ParameterReflection::
  lookupParameter(std::string const & name)
  {
    parameter_lookup_t::iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  Parameter const * ParameterReflection::
  lookupParameter(std::string const & name) const
  {
    parameter_lookup_t::const_iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    return ii->second;
  }
  
  
  Parameter * ParameterReflection::
  lookupParameter(std::string const & name, parameter_type_t type)
  {
    parameter_lookup_t::iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    if (type != ii->second->type_) {
      return 0;	      // could maybe implement some sort of casting...
    }
    return ii->second;
  }
  
  
  Parameter const * ParameterReflection::
  lookupParameter(std::string const & name, parameter_type_t type) const
  {
    parameter_lookup_t::const_iterator ii(parameter_lookup_.find(name));
    if (parameter_lookup_.end() == ii) {
      return 0;
    }
    if (type != ii->second->type_) {
      return 0;	      // could maybe implement some sort of casting...
    }
    return ii->second;
  }
  
  
  Status ParameterReflection::
  check(int const * param, int value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status ParameterReflection::
  check(std::string const * param, std::string const & value) const
  {
    Status ok;
    return ok;
  }


  Status ParameterReflection::
  check(double const * param, double value) const
  {
    Status ok;
    return ok;
  }

  
  Status ParameterReflection::
  check(Vector const * param, Vector const & value) const
  {
    Status ok;
    return ok;
  }
  
  
  Status ParameterReflection::
  check(Matrix const * param, Matrix const & value) const
  {
    Status ok; return ok;
  }
  
  
  void ParameterReflection::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty()) {
      os << title << "\n";
    }
    for (parameter_lookup_t::const_iterator ii(parameter_lookup_.begin());
	 ii != parameter_lookup_.end(); ++ii) {
      ii->second->dump(os, prefix + "    ");
    }
  }
  
  
  IntegerParameter * ParameterReflection::
  declareParameter(std::string const & name, int * integer, parameter_flags_t flags)
  {
    IntegerParameter * entry(new IntegerParameter(name, flags, this, integer));
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
    
  
  StringParameter * ParameterReflection::
  declareParameter(std::string const & name, std::string * instance, parameter_flags_t flags)
  {
    StringParameter * entry(new StringParameter(name, flags, this, instance));
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  RealParameter * ParameterReflection::
  declareParameter(std::string const & name, double * real, parameter_flags_t flags)
  {
    RealParameter * entry(new RealParameter(name, flags, this, real));
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  VectorParameter * ParameterReflection::
  declareParameter(std::string const & name, Vector * vector, parameter_flags_t flags)
  {
    VectorParameter * entry(new VectorParameter(name, flags, this, vector));
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  MatrixParameter * ParameterReflection::
  declareParameter(std::string const & name, Matrix * matrix, parameter_flags_t flags)
  {
    MatrixParameter * entry(new MatrixParameter(name, flags, this, matrix));
    parameter_lookup_.insert(std::make_pair(name, entry));
    return entry;
  }
  
  
  template<typename parameter_t, typename storage_t>
  bool maybe_append(std::vector<ParameterLog::log_s<parameter_t, storage_t> > & collection,
		    Parameter const * parameter)
  {
    parameter_t const * pp(dynamic_cast<parameter_t const *>(parameter));
    if (pp) {
      if (pp->flags_ & PARAMETER_FLAG_NOLOG) {
	return true;
      }
      collection.push_back(ParameterLog::log_s<parameter_t, storage_t>(pp));
      return true;
    }
    return false;
  }
  
  
  ParameterLog::
  ParameterLog(std::string const & nn, parameter_lookup_t const & parameter_lookup)
    : name(nn)
  {
    for (parameter_lookup_t::const_iterator ii(parameter_lookup.begin());
	 ii != parameter_lookup.end(); ++ii) {
      if (maybe_append(intlog, ii->second)) {
	continue;
      }
      if (maybe_append(strlog, ii->second)) {
	continue;
      }
      if (maybe_append(reallog, ii->second)) {
	continue;
      }
      if (maybe_append(veclog, ii->second)) {
	continue;
      }
      if (maybe_append(mxlog, ii->second)) {
	continue;
      }
    }
  }
  
  
  void ParameterLog::
  update(long long timestamp_)
  {
    timestamp.push_back(timestamp_);
    for (size_t ii(0); ii < intlog.size(); ++ii) {
      intlog[ii].log.push_back(*intlog[ii].parameter->getInteger());
    }
    for (size_t ii(0); ii < strlog.size(); ++ii) {
      strlog[ii].log.push_back(*strlog[ii].parameter->getString());
    }
    for (size_t ii(0); ii < reallog.size(); ++ii) {
      reallog[ii].log.push_back(*reallog[ii].parameter->getReal());
    }
    for (size_t ii(0); ii < veclog.size(); ++ii) {
      veclog[ii].log.push_back(*veclog[ii].parameter->getVector());
    }
    for (size_t ii(0); ii < mxlog.size(); ++ii) {
      mxlog[ii].log.push_back(*mxlog[ii].parameter->getMatrix());
    }
  }
  
  
  void ParameterLog::
  writeFiles(std::string const & prefix, std::ostream * progress) const
  {
    if (progress) {
      *progress << "writing parameter log: " << name << "\n";
    }
    
    if ( ! intlog.empty()) {
      if (progress) {
	*progress << "  integers:";
      }
      for (size_t ii(0); ii < intlog.size(); ++ii) {
	log_s<IntegerParameter, int> const & log(intlog[ii]);
	if ( ! log.log.empty()) {
	  if (progress) {
	    *progress << " " << log.parameter->name_ << "...";
	  }
	  std::string const fn(prefix + "-" + name + "-" + log.parameter->name_ + ".dump");
	  std::ofstream os(fn.c_str());
	  if (os) {
	    size_t const nn(log.log.size());
	    os << "# name: " << name << "\n"
	       << "# parameter: " << log.parameter->name_ << "\n"
	       << "# type: integer\n"
	       << "# size: " << nn << "\n";
	    for (size_t jj(0); jj < nn; ++jj) {
	      os << timestamp[jj] << "   " << log.log[jj] << "\n";
	    }
	  }
	}
      }
      if (progress) {
	*progress << " DONE\n";
      }
    }
    
    if ( ! strlog.empty()) {
      if (progress) {
	*progress << "  strings:";
      }
      for (size_t ii(0); ii < strlog.size(); ++ii) {
	log_s<StringParameter, std::string> const & log(strlog[ii]);
	if ( ! log.log.empty()) {
	  if (progress) {
	    *progress << " " << log.parameter->name_ << "...";
	  }
	  std::string const fn(prefix + "-" + name + "-" + log.parameter->name_ + ".dump");
	  std::ofstream os(fn.c_str());
	  if (os) {
	    size_t const nn(log.log.size());
	    os << "# name: " << name << "\n"
	       << "# parameter: " << log.parameter->name_ << "\n"
	       << "# type: string\n"
	       << "# size: " << nn << "\n";
	    for (size_t jj(0); jj < nn; ++jj) {
	      os << timestamp[jj] << "   " << log.log[jj] << "\n";
	    }
	  }
	}
      }
      if (progress) {
	*progress << " DONE\n";
      }
    }
    
    if ( ! reallog.empty()) {
      if (progress) {
	*progress << "  reals:";
      }
      for (size_t ii(0); ii < reallog.size(); ++ii) {
	log_s<RealParameter, double> const & log(reallog[ii]);
	if ( ! log.log.empty()) {
	  if (progress) {
	    *progress << " " << log.parameter->name_ << "...";
	  }
	  std::string const fn(prefix + "-" + name + "-" + log.parameter->name_ + ".dump");
	  std::ofstream os(fn.c_str());
	  if (os) {
	    size_t const nn(log.log.size());
	    os << "# name: " << name << "\n"
	       << "# parameter: " << log.parameter->name_ << "\n"
	       << "# type: real\n"
	       << "# size: " << nn << "\n";
	    for (size_t jj(0); jj < nn; ++jj) {
	      os << timestamp[jj] << "   " << log.log[jj] << "\n";
	    }
	  }
	}
      }
      if (progress) {
	*progress << " DONE\n";
      }
    }
    
    if ( ! veclog.empty()) {
      if (progress) {
	*progress << "  vectors:";
      }
      for (size_t ii(0); ii < veclog.size(); ++ii) {
	log_s<VectorParameter, Vector> const & log(veclog[ii]);
	if ( ! log.log.empty()) {
	  if (progress) {
	    *progress << " " << log.parameter->name_ << "...";
	  }
	  std::string const fn(prefix + "-" + name + "-" + log.parameter->name_ + ".dump");
	  std::ofstream os(fn.c_str());
	  if (os) {
	    size_t const nn(log.log.size());
	    os << "# name: " << name << "\n"
	       << "# parameter: " << log.parameter->name_ << "\n"
	       << "# type: vector\n"
	       << "# size: " << nn << "\n";
	    for (size_t jj(0); jj < nn; ++jj) {
	      os << timestamp[jj] << "   ";
	      jspace::pretty_print(log.log[jj], os, "", "");
	    }
	  }
	}
      }
      if (progress) {
	*progress << " DONE\n";
      }
    }
    
    if ( ! mxlog.empty()) {
      if (progress) {
	*progress << "  matrices:";
      }
      for (size_t ii(0); ii < mxlog.size(); ++ii) {
	log_s<MatrixParameter, Matrix> const & log(mxlog[ii]);
	if ( ! log.log.empty()) {
	  if (progress) {
	    *progress << " " << log.parameter->name_ << "...";
	  }
	  std::string const fn(prefix + "-" + name + "-" + log.parameter->name_ + ".dump");
	  std::ofstream os(fn.c_str());
	  if (os) {
	    size_t const nn(log.log.size());
	    os << "# name: " << name << "\n"
	       << "# parameter: " << log.parameter->name_ << "\n"
	       << "# type: matrix\n"
	       << "# size: " << nn << "\n"
	       << "# line format: tstamp nrows ncols row_0 row_1 ...\n";
	    for (size_t jj(0); jj < nn; ++jj) {
	      Matrix const & mx(log.log[jj]);
	      os << timestamp[jj] << "   " << mx.rows() << "  " << mx.cols();
	      for (int kk(0); kk < mx.rows(); ++kk) {
		os << "   ";
		for (int ll(0); ll < mx.cols(); ++ll) {
		  os << jspace::pretty_string(mx.coeff(kk, ll));
		}
	      }
	      os << "\n";
	    }
	  }
	}
      }
      if (progress) {
	*progress << " DONE\n";
      }
    }
    
  }
  
  
  void ReflectionRegistry::
  add(boost::shared_ptr<ParameterReflection> instance)
  {
    type_map_[instance->getTypeName()].insert(make_pair(instance->getName(), instance));
  }
  
  
  boost::shared_ptr<ParameterReflection> ReflectionRegistry::
  find(std::string const & type_name,
       std::string const & instance_name)
  {
    boost::shared_ptr<ParameterReflection> instance;
    type_map_t::iterator it(type_map_.find(type_name));
    if (type_map_.end() == it) {
      return instance;
    }
    instance_map_t::iterator ii(it->second.find(instance_name));
    if (it->second.end() == ii) {
      return instance;
    }
    instance = ii->second;
    return instance;
  }
  
  
  void ReflectionRegistry::
  enumerate(enumeration_t & enumeration)
  {
    enumeration_entry_s entry;
    for (type_map_t::iterator it(type_map_.begin()); type_map_.end() != it; ++it) {
      entry.type_name = it->first;
      for (instance_map_t::iterator ii(it->second.begin()); it->second.end() != ii; ++ii) {
	entry.instance_name = ii->first;
	parameter_lookup_t const & pt(ii->second->getParameterTable());
	for (parameter_lookup_t::const_iterator ip(pt.begin()); pt.end() != ip; ++ip) {
	  entry.parameter_name = ip->first;
	  entry.parameter = ip->second;
	  enumeration.push_back(entry);
	}
      }
    }
  }
  
  
  Parameter * ReflectionRegistry::
  lookupParameter(std::string const & type_name,
		  std::string const & instance_name,
		  std::string const & parameter_name)
  {
    boost::shared_ptr<ParameterReflection> ref(find(type_name, instance_name));
    if ( ! ref) {
      return 0;
    }
    return ref->lookupParameter(parameter_name);
  }
  
  
  Parameter const * ReflectionRegistry::
  lookupParameter(std::string const & type_name,
		  std::string const & instance_name,
		  std::string const & parameter_name) const
  {
    boost::shared_ptr<ParameterReflection const>
      ref(const_cast<ReflectionRegistry*>(this)->find(type_name, instance_name));
    if ( ! ref) {
      return 0;
    }
    return ref->lookupParameter(parameter_name);
  }
  
  
  Parameter * ReflectionRegistry::
  lookupParameter(std::string const & type_name,
		  std::string const & instance_name,
		  std::string const & parameter_name,
		  parameter_type_t parameter_type)
  {
    boost::shared_ptr<ParameterReflection> ref(find(type_name, instance_name));
    if ( ! ref) {
      return 0;
    }
    return ref->lookupParameter(parameter_name, parameter_type);
  }
  
  
  Parameter const * ReflectionRegistry::
  lookupParameter(std::string const & type_name,
		  std::string const & instance_name,
		  std::string const & parameter_name,
		  parameter_type_t parameter_type) const
  {
    boost::shared_ptr<ParameterReflection const>
      ref(const_cast<ReflectionRegistry*>(this)->find(type_name, instance_name));
    if ( ! ref) {
      return 0;
    }
    return ref->lookupParameter(parameter_name, parameter_type);
  }

}
