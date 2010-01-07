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

/** \file testmisc/TaskAtomizer.cpp Unit test of class TaskAtomizer. */

#include <wbcnet/misc/TaskAtomizer.hpp>
#include <wbcnet/imp/SPQueue.hpp>
#include <wbcnet/data.hpp>
#include <wbcnet/proxy.hpp>
#include <wbcnet/Muldex.hpp>
#include <wbcnet/log.hpp>
#include <wbcnet/msg/TaskMatrix.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <err.h>

using namespace wbcnet;
using namespace std;

static endian_mode_t const endian_mode(ENDIAN_DETECT);
static unique_id_t matrix_message_id;
static uint8_t const max_n_rows_cols(std::numeric_limits<uint8_t>::max());

namespace {
  
  typedef vector<Matrix<double> *> matrix_list_t;
  typedef vector<matrix_list_t> matrix_array_t;
  
  class Listener: public MdxListener {
  public:
    Listener(TaskAtomizer & atomizer, bool before, ostream & os);
    ~Listener();
    
    virtual int HandleMessageHeader(unique_id_t msg_id);
    virtual int HandleMessagePayload(unique_id_t msg_id);
    
    uint8_t GetNIndepMatrices() const;
    int8_t GetNSets() const;
    uint8_t GetNSetMatrices() const;
    int8_t GetTTNTasks() const;
    uint8_t GetNTaskMatrices() const;
    Matrix<double> * GetMatrix(int8_t setID, int8_t taskID, uint8_t matrixID);
    
    bool SelectAndSetMatrix(uint8_t requestID, int8_t setID, int8_t taskID, uint8_t matrixID);
    
    TaskAtomizer & atomizer;
    ostream & os;
    timestamp acquisition_time;
    matrix_list_t indep_matrix;
    matrix_array_t set_matrix;
    matrix_array_t task_matrix;
    msg::TaskMatrixWrap matrix_wrap;
  };
  
}


static Matrix<double> * CreateMatrix(uint8_t n_rows, uint8_t n_columns)
{
  struct timeval now;
  if (0 != gettimeofday(&now, 0))
    err(EXIT_FAILURE, "in CreateMatrix(): gettimeofday()");
  Matrix<double> * mx(new Matrix<double>(n_rows, n_columns));
  for (uint8_t ir(0); ir < n_rows; ++ir)
    for (uint8_t ic(0); ic < n_columns; ++ic)
      mx->GetElement(ir, ic) = now.tv_usec + ir * (int) n_columns + ic;
  return mx;
}


static void cleanup()
{
  idl::DestroySingleton();
}


static bool send_matrices(SPQueue & queue, uint8_t requestID, Listener & listener, ostream & os)
{
  MdxDispatcher mdx(0, -1, endian_mode);
  listener.matrix_wrap.requestID = requestID;
  
  cout << "send_matrices():\n task-independent";
  uint8_t nmx(listener.GetNIndepMatrices());
  for (uint8_t imx(0); imx < nmx; ++imx) {
    cout << " " << (int) imx << flush;
    if ( ! listener.SelectAndSetMatrix(requestID, -1, -1, imx)) {
      os << "  send_matrices() FAILURE: listener.SelectAndSetMatrix(-1, -1, " << (int) imx << ") FAILED\n";
      cout << "ERROR\n";
      return false;
    }
    muldex_status const ms(mdx.Mux(&queue, listener.matrix_wrap));
    if (muldex_status::SUCCESS != ms.muldex) {
      os << "  send_matrices() FAILURE: mdx.Mux() FAILED on task-independent matrix ID "
	 << (int) imx << ": " << muldex_status_str(ms) << "\n";
      cout << "ERROR\n";
      return false;
    }
  }
  
  cout << "\n set-dependent\n";
  nmx = listener.GetNSetMatrices();
  for (int8_t iset(0); iset < listener.GetNSets(); ++iset) {
    cout << "  set " << (int) iset << ":" << flush;
    for (uint8_t imx(0); imx < nmx; ++imx) {
      cout << " " << (int) imx << flush;
      if ( ! listener.SelectAndSetMatrix(requestID, iset, -1, imx)) {
	os << "  send_matrices() FAILURE: listener.SelectAndSetMatrix(" << (int) iset << ", -1, "
	   << (int) imx << ") FAILED\n";
	cout << "ERROR\n";
	return false;
      }
      muldex_status const ms(mdx.Mux(&queue, listener.matrix_wrap));
      if (muldex_status::SUCCESS != ms.muldex) {
	os << "  send_matrices() FAILURE: mdx.Mux() FAILED on set ID " << (int) iset << " matrix ID "
	   << (int) imx << ": " << muldex_status_str(ms) << "\n";
	cout << "ERROR\n";
	return false;
      }
    }
    cout << "\n";
  }
  
  cout << " task-dependent\n";
  nmx = listener.GetNTaskMatrices();
  for (int8_t itask(-1); itask < listener.GetTTNTasks(); ++itask) {
    cout << "  task " << (int) itask << ":" << flush;
    for (uint8_t imx(0); imx < nmx; ++imx) {
      cout << " " << (int) imx << flush;
      if ( ! listener.SelectAndSetMatrix(requestID, 0, itask, imx)) {
	os << "  send_matrices() FAILURE: listener.SelectAndSetMatrix(0, " << (int) itask << ", "
	   << (int) imx << ") FAILED\n";
	cout << "ERROR\n";
	return false;
      }
      muldex_status const ms(mdx.Mux(&queue, listener.matrix_wrap));
      if (muldex_status::SUCCESS != ms.muldex) {
	os << "  send_matrices() FAILURE: mdx.Mux() FAILED on task ID " << (int) itask << " matrix ID "
	   << (int) imx << ": " << muldex_status_str(ms) << "\n";
	cout << "ERROR\n";
	return false;
      }
    }
    cout << "\n";
  }
  
  return true;
}


static int compare(Listener & before, Listener & after,
		   ostream & cmp_os, ostream & err_os)
{
  if (before.indep_matrix.size() != after.indep_matrix.size()) {
    err_os << "compare(): before.indep_matrix.size() != after.indep_matrix.size()\n";
    return -1;
  }
  
  bool ok(true);
  int count(0);
  for (uint8_t imx(0); imx < before.indep_matrix.size(); ++imx) {
    Matrix<double> const * mbefore(before.GetMatrix(-1, -1, imx));
    Matrix<double> const * mafter(after.GetMatrix(-1, -1, imx));
    if (mbefore && mafter) {
      if (*mbefore == *mafter)
	cmp_os << "e" << flush;
      else {
	++count;
	cmp_os << "n" << flush;
      }
    }
    else {
      ok = false;
      cmp_os << "x" << flush;
    }
  }
  if ( ! ok) {
    err_os << "compare(): indep_matrix did not exist (see above)\n";
    return -2;
  }
  
  ok = true;
  for (int8_t iset(0); iset < static_cast<int8_t>(before.set_matrix.size()); ++iset) {
    for (uint8_t imx(0); imx < before.set_matrix[iset].size(); ++imx) {
      Matrix<double> const * mbefore(before.GetMatrix(iset, -1, imx));
      Matrix<double> const * mafter(after.GetMatrix(iset, -1, imx));
      if (mbefore && mafter) {
	if (*mbefore == *mafter)
	  cmp_os << "E" << flush;
	else {
	  ++count;
	  cmp_os << "N" << flush;
	}
      }
      else {
	ok = false;
	cmp_os << "X" << flush;
      }
    }
  }
  if ( ! ok) {
    err_os << "compare(): set_matrix did not exist (see above)\n";
    return -3;
  }
  
  ok = true;
  for (int8_t itask(0); itask < static_cast<int8_t>(before.task_matrix.size()); ++itask) {
    for (uint8_t imx(0); imx < before.task_matrix[itask].size(); ++imx) {
      Matrix<double> const * mbefore(before.GetMatrix(0, itask, imx));
      Matrix<double> const * mafter(after.GetMatrix(0, itask, imx));
      if (mbefore && mafter) {
	if (*mbefore == *mafter)
	  cmp_os << "E" << flush;
	else {
	  ++count;
	  cmp_os << "N" << flush;
	}
      }
      else {
	ok = false;
	cmp_os << "X" << flush;
      }
    }
  }
  if ( ! ok) {
    err_os << "compare(): task_matrix did not exist (see above)\n";
    return -4;
  }
  
  return count;
}


static bool test(TaskAtomizer & mta, ostream & os)
{
  uint8_t const requestID(42);
  
  SPQueue queue;
  Listener before(mta, true, os);
  Listener after(mta, false, os);
  if (send_matrices(queue, requestID, before, os))
    cout << "test(): send_matrices() passed\n";
  else {
    os << "  FAILURE: send_matrices() FAILED\n";
    return false;
  }
  
  TaskAtomizer::status const st(mta.Reset(requestID,
					  before.GetNIndepMatrices(),
					  before.GetNSets(),
					  before.GetNSetMatrices(),
					  before.GetTTNTasks(),
					  before.GetNTaskMatrices()));
  if (TaskAtomizer::OK == st)
    cout << "test(): mta.Reset() passed\n";
  else {
    os << "  FAILURE: mta.Reset() FAILED with status " << st << " \""
       << TaskAtomizer::StatusString(st) << "\"\n";
    return false;
  }
  
  MdxDispatcher dispatcher(0, -1, endian_mode);
  dispatcher.SetHandler(matrix_message_id,
			new ProxyHandler("matrix", after.matrix_wrap, true, &after));
  
  for (int ii(0); ! queue.Empty(); ++ii) {
    
    if (mta.ModelsAreComplete()) {
      os << "  FAILURE: mta.ModelsAreComplete() is true at top of loop\n";
      return false;
    }
    
    cout << "  before demuxing [" << ii << "]: " << flush;
    int difcount(compare(before, after, cout, os));
    cout << "\n    difcount " << difcount << "\n";
    if (0 > difcount) {
      os << "  FAILURE during comparison before demuxing message " << ii << " (see above)\n";
      return false;
    }
    
    muldex_status const ms(dispatcher.DemuxOne(&queue));
    if (muldex_status::SUCCESS != ms.muldex) {
      os << "  FAILURE: dispatcher.DemuxOne() for message " << ii << "\n";
      return false;
    }
    
    cout << "  after demuxing [" << ii << "]: " << flush;
    difcount = compare(before, after, cout, os);
    cout << "\n    difcount " << difcount << "\n";
    if (0 > difcount) {
      os << "  FAILURE during comparison before demuxing message " << ii << " (see above)\n";
      return false;
    }
    
    if (mta.ModelsAreComplete())
      cout << "test(): models are complete, hope this was the last msg...\n";
  }
  
  if (mta.ModelsAreComplete())
    cout << "test(): model completion passed\n";
  else {
    os << "  FAILURE: mta.ModelsAreComplete() is false after last message\n";
    return false;
  }
  
  return true;
}


int main(int argc, char ** argv)
{
  wbcnet::configure_logging();
  if (0 != atexit(cleanup))
    errx(EXIT_FAILURE, "atexit() failed");
  idl::CreateSingleton();
  
  matrix_message_id = idl::Assign("matrix");
  
  cout << "task atomizer capacity: " << TaskAtomizer::capacity << "\n";
  
  TaskAtomizer mta(endian_mode);
  ostringstream ttos;
  if ( ! test(mta, ttos)) {
    cout << "detected FAILURES\n"
	 << ttos.str();
    return EXIT_FAILURE;    
  }
  cout << "\nALL TESTS PASSED\n\n";
  return EXIT_SUCCESS;
}


namespace {
  
  
  Listener::
  Listener(TaskAtomizer & _atomizer, bool before, ostream & _os):
    atomizer(_atomizer),
    os(_os),
    matrix_wrap(idl::GetID("matrix"), max_n_rows_cols, max_n_rows_cols, 0)
  {
    if ( ! acquisition_time.gettimeofday(0))
      err(EXIT_FAILURE, "gettimeofday()");
    if (before) {
      indep_matrix.push_back(CreateMatrix(3, 4));
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(7, 3));
	ml.push_back(CreateMatrix(2, 5));
	ml.push_back(CreateMatrix(1, 1));
	set_matrix.push_back(ml);
      }
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(1, 2));
	ml.push_back(CreateMatrix(2, 1));
	task_matrix.push_back(ml);
      }
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(2, 2));
	ml.push_back(CreateMatrix(3, 4));
	task_matrix.push_back(ml);
      }
    }
    else {
      indep_matrix.push_back(CreateMatrix(4, 3));
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(3, 7));
	ml.push_back(CreateMatrix(5, 2));
	ml.push_back(CreateMatrix(3, 1));
	set_matrix.push_back(ml);
      }
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(2, 2));
	ml.push_back(CreateMatrix(3, 4));
	task_matrix.push_back(ml);
      }
      {
	matrix_list_t ml;
	ml.push_back(CreateMatrix(1, 2));
	ml.push_back(CreateMatrix(2, 1));
	task_matrix.push_back(ml);
      }
    }
  }
  
  
  Listener::
  ~Listener()
  {
    for (matrix_list_t::iterator im(indep_matrix.begin()); im != indep_matrix.end(); ++im)
      delete *im;
    for (matrix_array_t::iterator ima(set_matrix.begin()); ima != set_matrix.end(); ++ima)
      for (matrix_list_t::iterator im(ima->begin()); im != ima->end(); ++im)
	delete *im;
    for (matrix_array_t::iterator ima(task_matrix.begin()); ima != task_matrix.end(); ++ima)
      for (matrix_list_t::iterator im(ima->begin()); im != ima->end(); ++im)
	delete *im;
  }
  
  
  int Listener::
  HandleMessageHeader(unique_id_t msg_id)
  {
    TaskAtomizer::status const st(atomizer.ProcessHeader(matrix_wrap));
    if (TaskAtomizer::OK != st) {
      os << "Listener::HandleMessageHeader(): atomizer.ProcessHeader() failed\n"
	 << "  reason: " << atomizer.StatusString(st) << "\n";
      return 17;
    }
    Matrix<double> * matrix(GetMatrix(matrix_wrap.setID, matrix_wrap.taskID, matrix_wrap.matrixID));
    if ( ! matrix) {
      os << "Listener::HandleMessageHeader(): GetMatrix(" << (int) matrix_wrap.setID
	 << ", " << (int) matrix_wrap.taskID << ", " << (int) matrix_wrap.matrixID << ") failed\n";
      return 18;
    }
    matrix_wrap.dataptr = matrix;
    return 0;
  }
  
  
  int Listener::
  HandleMessagePayload(unique_id_t msg_id)
  {
    TaskAtomizer::status const st(atomizer.ProcessPayload(matrix_wrap));
    if (TaskAtomizer::OK != st) {
      os << "Listener::HandleMessagePayload(): atomizer.ProcessPayload() failed\n"
	 << "  reason: " << atomizer.StatusString(st) << "\n";
      return 18;
    }
    return 0;
  }
  
  
  bool Listener::
  SelectAndSetMatrix(uint8_t requestID, int8_t setID, int8_t taskID, uint8_t matrixID)
  {
    if ((setID < -1) || (setID >= GetNSets())) {
      os << "Listener::SelectAndSetMatrix(): invalid setID = " << (int) setID << "\n";
      return false;
    }
    if ((taskID < -1) || (taskID >= GetTTNTasks())) {
      os << "Listener::SelectAndSetMatrix(): invalid taskID = " << (int) taskID << "\n";
      return false;
    }
    Matrix<double> * matrix(GetMatrix(setID, taskID, matrixID));
    if ( ! matrix) {
      os << "Listener::SelectAndSetMatrix(): GetMatrix(" << (int) taskID
	 << ", " << (int) matrixID << ") failed\n";
      return false;
    }
    matrix_wrap.acquisitionTime = acquisition_time;
    matrix_wrap.setID = setID;
    matrix_wrap.taskID = taskID;
    matrix_wrap.matrixID = matrixID;
    matrix_wrap.nRows = matrix->NRows();
    matrix_wrap.nColumns = matrix->NColumns();
    matrix_wrap.dataptr = matrix;
    return true;
  }
  
  
  uint8_t Listener::
  GetNIndepMatrices() const
  {
    return indep_matrix.size() & 0xFF;
  }
  
  
  int8_t Listener::
  GetNSets() const
  {
    return set_matrix.size() & 0x7F;
  }
  
  
  uint8_t Listener::
  GetNSetMatrices() const
  {
    if (set_matrix.empty())
      return 0;
    return set_matrix[0].size() & 0xFF;
  }
  
  
  int8_t Listener::
  GetTTNTasks() const
  {
    return task_matrix.size() & 0x7F;
  }
  
  
  uint8_t Listener::
  GetNTaskMatrices() const
  {
    if (task_matrix.empty())
      return 0;
    return task_matrix[0].size() & 0xFF;
  }
  
  
  Matrix<double> * Listener::
  GetMatrix(int8_t setID, int8_t taskID, uint8_t matrixID)
  {
    Matrix<double> * matrix(0);
    if (-1 == taskID) {
      if (-1 == setID) {
	if (matrixID < indep_matrix.size())
	  matrix = indep_matrix[matrixID];
      }
      else if ((setID >= 0) && (setID < static_cast<int8_t>(set_matrix.size()))) {
	if (matrixID < set_matrix[setID].size())
	  matrix = set_matrix[setID][matrixID];
      }
    }
    else if ((taskID >= 0) && (taskID < static_cast<int8_t>(task_matrix.size()))) {
      if (matrixID < task_matrix[taskID].size())
	matrix = task_matrix[taskID][matrixID];
    }
    return matrix;
  }
  
}
