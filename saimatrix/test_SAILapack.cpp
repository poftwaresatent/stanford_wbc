/*
 * saimatrix -- utility library for stanford-wbc.sourceforge.net
 *
 * Copyright (c) 1997-2008 Stanford University. All rights reserved.
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

#include <iostream>
#include <sstream>
#include "SAIMatrix.h"
#include "SAILapack.h"
#include <err.h>

using namespace std;

// The SVD example is copy-pasted from Wikipedia, thank you very much!
// http://en.wikipedia.org/wiki/Singular_value_decomposition
// ...but somehow, the results differ...

static double const tstA[] = {
  1, 0, 0, 0, 2,
  0, 0, 3, 0, 0,
  0, 0, 0, 0, 0,
  0, 4, 0, 0, 0
};

static double const tstU[] = {
  0, 0, 1,  0,
  0, 1, 0,  1,
  0, 0, 0, -1,
  1, 0, 0,  0
};

static double const sqrt5(2.2360679775); // sqrt(5)
static double const tstS[] = {
  4, 0,     0, 0, 0,
  0, 3,     0, 0, 0,
  0, 0, sqrt5, 0, 0,
  0, 0,     0, 0, 0
};

static double const sqrt02(0.4472135955); // sqrt(0.2)
static double const sqrt08(0.894427191);  // sqrt(0.8)
static double const tstV[] = {
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
    sqrt02, 0, 0, 0, sqrt08,
         0, 0, 0, 1, 0,
  - sqrt08, 0, 0, 0, sqrt02
};


int main(int argc, char ** argv)
{
  SAIMatrix const chkA(tstA, 4, 5, false);
  SAIMatrix const chkU(tstU, 4, 4, false);
  SAIMatrix const chkS(tstS, 4, 5, false);
  SAIMatrix const chkV(tstV, 5, 5, false);
  
  SAIMatrix actU, actS, actV;
  int const result(sai_lapack_svd(chkA, actU, actS, actV));
  if (0 != result)
    errx(1, "sai_lapack_svd() returned %d", result);
  
  ostringstream erros;
  bool ok(true);
  
  cout << "==================================================\n"
       << "checking A\n"
       << "--------------------------------------------------\n";
  SAIMatrix const actA(actU * actS * actV.transpose());
  SAIMatrix const difA(chkA - actA);
  chkA.display("desired A");
  actA.display("actual A");
  difA.display("delta A");
  double sum(0);
  for (int ii(0); ii < difA.row(); ii++)
    for (int jj(0); jj < difA.column(); jj++)
      sum += fabs(difA.elementAt(ii, jj));
  cout << "--------------------------------------------------\n"
       << "sum of absolute delta elements: " << sum << "\n"
       << "==================================================\n";
  if (sum > 1e-9) {
    ok = false;
    erros << "sum of absolute delta elements of A: " << sum << "\n";
  }
  
  cout << "checking that U^T * U is identity\n"
       << "--------------------------------------------------\n";
  SAIMatrix const UtU(actU.transpose() * actU);
  actU.display("U");
  UtU.display("U^T * U");
  sum = 0;
  for (int ii(0); ii < UtU.row(); ii++)
    for (int jj(0); jj < UtU.column(); jj++)
      if (ii == jj)
	sum += fabs(UtU.elementAt(ii, jj) - 1);
      else
	sum += fabs(UtU.elementAt(ii, jj));
  cout << "--------------------------------------------------\n"
       << "sum of absolute delta elements: " << sum << "\n"
       << "==================================================\n";
  if (sum > 1e-9) {
    ok = false;
    erros << "U^T * U is not identity, sum of absolute delta elements: " << sum << "\n";
  }
  
  cout << "checking that Sigma is non-negative diagonal\n"
       << "--------------------------------------------------\n";
  actS.display("actual S");
  cout << "--------------------------------------------------\n";
  for (int ii(0); ii < actS.row(); ii++)
    for (int jj(0); jj < actS.column(); jj++)
      if (ii == jj) {
	if (actS.elementAt(ii, jj) < 0) {
	  ok = false;
	  cout << "  S[" << ii << "][" << jj << "] is negative\n";
	  erros << "  S[" << ii << "][" << jj << "] is negative\n";
	}
      }
      else {
	if (fabs(actS.elementAt(ii, jj)) > 1e-9) {
	  ok = false;
	  cout << "  S[" << ii << "][" << jj << "] is non-zero\n";
	  erros << "  S[" << ii << "][" << jj << "] is non-zero\n";
	}
      }
  cout << "==================================================\n";
  
  actV.display("actual V");
  cout << "==================================================\n"
       << erros.str();
  
  if ( ! ok)
    errx(1, "test FAILED");
  cout << "test succeeed\n";
}
