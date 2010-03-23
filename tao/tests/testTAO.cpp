/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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

/**
   \file testTAO.cpp
   \author Roland Philippsen
*/

#include <tao/utility/TaoDeMassProp.h>
#include <gtest/gtest.h>

using namespace std;


TEST (mass_prop, point_inertias)
{
  static double const pm[3] = {  1.4,  0.7, 5.1 }; // point masses
  static double const px[3] = {  1.2,  0.1, 0.3 }; // point x-coords
  static double const py[3] = { -0.1,  1.4, 0.2 };
  static double const pz[3] = {  0.1,  0.1, 0.9 };
  
  // center of mass
  double com[3] = { 0, 0, 0 };
  double M(0);
  for (int ii(0); ii < 3; ++ii) {
    com[0] += pm[ii] * px[ii];
    com[1] += pm[ii] * py[ii];
    com[2] += pm[ii] * pz[ii];
    M += pm[ii];
  }
  com[0] /= M;
  com[1] /= M;
  com[2] /= M;
  
  // moments
  double Ixx(0);
  double Iyy(0);
  double Izz(0);
  double Ixy(0);
  double Iyz(0);
  double Izx(0);
  for (int ii(0); ii < 3; ++ii) {
    Ixx += pm[ii] * (pow(py[ii], 2) + pow(pz[ii], 2));
    Iyy += pm[ii] * (pow(pz[ii], 2) + pow(px[ii], 2));
    Izz += pm[ii] * (pow(px[ii], 2) + pow(py[ii], 2));
    Ixy += pm[ii] * px[ii] * py[ii];
    Iyz += pm[ii] * py[ii] * pz[ii];
    Izx += pm[ii] * pz[ii] * px[ii];
  }
  
  // TAO sum of point masses
  deMassProp check;
  for (int ii(0); ii < 3; ++ii) {
    deFrame const foo(px[ii], py[ii], pz[ii]);
    check.mass(pm[ii], &foo);
  }
  
  deFloat check_mass;
  deVector3 check_center;
  deMatrix3 check_inertia;
  check.get(&check_mass, &check_center, &check_inertia);
  
  EXPECT_LT (fabs(check_mass - M), 1e-6)
    << "total mass: expected = " << M << " received = " << check_mass;
  for (int ii(0); ii < 3; ++ii) {
    EXPECT_LT (fabs(check_center[ii] - com[ii]), 1e-6)
      << "com[" << ii << "]: expected = " << com[ii] << " received = " << check_center[ii];
  }
  
  EXPECT_LT (fabs(check_inertia.elementAt(0, 0) - Ixx), 1e-6)
    << "Ixx: expected = " << Ixx << " received = " << check_inertia.elementAt(0, 0);
  EXPECT_LT (fabs(check_inertia.elementAt(1, 1) - Iyy), 1e-6)
    << "Iyy: expected = " << Iyy << " received = " << check_inertia.elementAt(1, 1);
  EXPECT_LT (fabs(check_inertia.elementAt(2, 2) - Izz), 1e-6)
    << "Izz: expected = " << Izz << " received = " << check_inertia.elementAt(2, 2);
  
  // The off-diagonal terms are counted negative, but above we
  // calculated just the scalar expressions, so add instead of
  // substract here.
  EXPECT_LT (fabs(check_inertia.elementAt(0, 1) + Ixy), 1e-6)
    << "Ixy: expected = " << - Ixy << " received = " << check_inertia.elementAt(0, 1);
  EXPECT_LT (fabs(check_inertia.elementAt(1, 2) + Iyz), 1e-6)
    << "Iyz: expected = " << - Iyz << " received = " << check_inertia.elementAt(1, 2);
  EXPECT_LT (fabs(check_inertia.elementAt(0, 2) + Izx), 1e-6)
    << "Izx: expected = " << - Izx << " received = " << check_inertia.elementAt(0, 2);
}


TEST (mass_prop, translate_inertia)
{
  static double const pm[3] = {  1.4,  0.7, 5.1 }; // point masses
  static double const px[3] = {  1.2,  0.1, 0.3 }; // point x-coords
  static double const py[3] = { -0.1,  1.4, 0.2 };
  static double const pz[3] = {  0.1,  0.1, 0.9 };
  
  static double const delta[3] = { // offset of px, py, pz
     0.8,
     1.5,
    -2.7
  };
  
  double com_o[3] = { 0, 0, 0 }; // center of mass without shift
  double com_d[3] = { 0, 0, 0 }; // COM shifted by delta[]
  double M(0);
  for (int ii(0); ii < 3; ++ii) {
    com_o[0] += pm[ii] * px[ii];
    com_o[1] += pm[ii] * py[ii];
    com_o[2] += pm[ii] * pz[ii];
    
    com_d[0] += pm[ii] * (px[ii] + delta[0]);
    com_d[1] += pm[ii] * (py[ii] + delta[1]);
    com_d[2] += pm[ii] * (pz[ii] + delta[2]);
    
    M += pm[ii];
  }
  for (int ii(0); ii < 3; ++ii) {
    com_o[ii] /= M;
    com_d[ii] /= M;
  }
  
  // moments around COM, will be used to feed TAO
  double Ixx_c(0);
  double Iyy_c(0);
  double Izz_c(0);
  double Ixy_c(0);
  double Iyz_c(0);
  double Izx_c(0);
  // moments directly around the shifted origin, will be used for comparison
  double Ixx_d(0);
  double Iyy_d(0);
  double Izz_d(0);
  double Ixy_d(0);
  double Iyz_d(0);
  double Izx_d(0);
  for (int ii(0); ii < 3; ++ii) {
    Ixx_c += pm[ii] * (pow(py[ii] - com_o[1], 2) + pow(pz[ii] - com_o[2], 2));
    Iyy_c += pm[ii] * (pow(pz[ii] - com_o[2], 2) + pow(px[ii] - com_o[0], 2));
    Izz_c += pm[ii] * (pow(px[ii] - com_o[0], 2) + pow(py[ii] - com_o[1], 2));
    
    Ixy_c += pm[ii] * (px[ii] - com_o[0]) * (py[ii] - com_o[1]);
    Iyz_c += pm[ii] * (py[ii] - com_o[1]) * (pz[ii] - com_o[2]);
    Izx_c += pm[ii] * (pz[ii] - com_o[2]) * (px[ii] - com_o[0]);

    Ixx_d += pm[ii] * (pow(py[ii] + delta[1], 2) + pow(pz[ii] + delta[2], 2));
    Iyy_d += pm[ii] * (pow(pz[ii] + delta[2], 2) + pow(px[ii] + delta[0], 2));
    Izz_d += pm[ii] * (pow(px[ii] + delta[0], 2) + pow(py[ii] + delta[1], 2));
    
    Ixy_d += pm[ii] * (px[ii] + delta[0]) * (py[ii] + delta[1]);
    Iyz_d += pm[ii] * (py[ii] + delta[1]) * (pz[ii] + delta[2]);
    Izx_d += pm[ii] * (pz[ii] + delta[2]) * (px[ii] + delta[0]);
  }
  
  deMassProp check;
  
  // TAO mass at shifted COM
  deFrame const comframe(com_d[0], com_d[1], com_d[2]);
  check.mass(M, &comframe);
  
  // TAO inertia around COM shifted by (delta[] - COM)
  deFrame const deltaframe(delta[0] - com_o[0],
			   delta[1] - com_o[1],
			   delta[2] - com_o[2]);
  deMatrix3 I_c;
  I_c.set( Ixx_c, -Ixy_c, -Izx_c,
	  -Ixy_c,  Iyy_c, -Iyz_c,
	  -Izx_c, -Iyz_c,  Izz_c);
  check.inertia(&I_c, &deltaframe);
  
  deFloat check_mass;
  deVector3 check_center;
  deMatrix3 check_inertia;
  check.get(&check_mass, &check_center, &check_inertia);
  
  EXPECT_LT (fabs(check_mass - M), 1e-6)
    << "total mass: expected = " << M << " received = " << check_mass;
  for (int ii(0); ii < 3; ++ii) {
    EXPECT_LT (fabs(check_center[ii] - com_d[ii]), 1e-6)
      << "com[" << ii << "]: expected = " << com_d[ii] << " received = " << check_center[ii];
  }
  
  EXPECT_LT (fabs(check_inertia.elementAt(0, 0) - Ixx_d), 1e-6)
    << "Ixx: expected = " << Ixx_d << " received = " << check_inertia.elementAt(0, 0);
  EXPECT_LT (fabs(check_inertia.elementAt(1, 1) - Iyy_d), 1e-6)
    << "Iyy: expected = " << Iyy_d << " received = " << check_inertia.elementAt(1, 1);
  EXPECT_LT (fabs(check_inertia.elementAt(2, 2) - Izz_d), 1e-6)
    << "Izz: expected = " << Izz_d << " received = " << check_inertia.elementAt(2, 2);
  
  EXPECT_LT (fabs(check_inertia.elementAt(0, 1) + Ixy_d), 1e-6)
    << "Ixy: expected = " << - Ixy_d << " received = " << check_inertia.elementAt(0, 1);
  EXPECT_LT (fabs(check_inertia.elementAt(1, 2) + Iyz_d), 1e-6)
    << "Iyz: expected = " << - Iyz_d << " received = " << check_inertia.elementAt(1, 2);
  EXPECT_LT (fabs(check_inertia.elementAt(0, 2) + Izx_d), 1e-6)
    << "Izx: expected = " << - Izx_d << " received = " << check_inertia.elementAt(0, 2);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
