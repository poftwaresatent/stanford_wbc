/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 1997-2010 Stanford University. All rights reserved.
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

// *******************************************************************
// pumaDynamics.cpp
//
// This file calculates the matrices needed for puma dynamics in joint
// space, including the Jacobian, mass matrix, coriolis/centrifugal
// vector, and gravity vector.  The dJ matrix is the derivative dJ/dt,
// used to convert the B matrix to operational space.
// *******************************************************************

#include "param.h"
#include <saimatrix/SAIMatrix6.h>
#include "pumaDynamics.h"

void getPumaDynamics( const SAIVector& q, const SAIVector& dq,
                      SAIMatrix& J, SAIMatrix& dJ,
                      SAIMatrix& A, SAIVector& B, SAIVector& G )
{
   Float q1 = q[0];
   Float q2 = q[1];
   Float q3 = q[2];
   Float q4 = q[3];
   Float q5 = q[4];
   // Float q6 = q[5];

   Float dq1 = dq[0];
   Float dq2 = dq[1];
   Float dq3 = dq[2];
   Float dq4 = dq[3];
   Float dq5 = dq[4];
   Float dq6 = dq[5];

   Float c1 = cos( q1 );
   Float c2 = cos( q2 );
   Float c3 = cos( q3 );
   Float c4 = cos( q4 );
   Float c5 = cos( q5 );
   Float c22 = cos( q2 + q2 );
   Float c23 = cos( q2 + q3 );
   Float c2323 = cos( 2 * (q2 + q3) );
   Float c44 = cos( q4 + q4 );
   Float c55 = cos( q5 + q5 );

   Float s1 = sin( q1 );
   Float s2 = sin( q2 );
   Float s3 = sin( q3 );
   Float s4 = sin( q4 );
   Float s5 = sin( q5 );
   Float s22 = sin( q2 + q2 );
   Float s23 = sin( q2 + q3 );
   Float s2323 = sin( 2 * (q2 + q3) );
   Float s44 = sin( q4 + q4 );
   Float s55 = sin( q5 + q5 );

   // ****************************************************************
   // Jacobian
   // ****************************************************************

   J.setSize( 6, 6 );

   J[0][0] = -s1*(L2*c2 + (L3+L6*c5)*s23 + L6*c23*c4*s5) - c1*(L1 + L6*s4*s5);
   J[0][1] = c1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   J[0][2] = c1*(c23*(L3 + L6*c5) - L6*c4*s23*s5);
   J[0][3] = -L6*(c4*s1 + c1*c23*s4)*s5;
   J[0][4] = -L6*(c5*s1*s4 + c1*(-c23*c4*c5 + s23*s5));
   J[0][5] = 0;

   J[1][0] = c1*(L2*c2 + (L3 + L6*c5)*s23 + L6*c23*c4*s5) - s1*(L1 + L6*s4*s5);
   J[1][1] = s1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   J[1][2] = s1*(c23*(L3 + L6*c5) - L6*c4*s23*s5);
   J[1][3] = L6*(c1*c4 - c23*s1*s4)*s5;
   J[1][4] = L6*(c23*c4*c5*s1 + c1*c5*s4 - s1*s23*s5);
   J[1][5] = 0;

   J[2][0] = 0;
   J[2][1] = -L2*c2 - (L3 + L6*c5)*s23 - L6*c23*c4*s5;
   J[2][2] = -(L3 + L6*c5)*s23 - L6*c23*c4*s5;
   J[2][3] = L6*s23*s4*s5;
   J[2][4] = -L6*(c4*c5*s23 + c23*s5);
   J[2][5] = 0;

   J[3][0] = 0;
   J[3][1] = -s1;
   J[3][2] = -s1;
   J[3][3] = c1*s23;
   J[3][4] = -c4*s1 - c1*c23*s4;
   J[3][5] = -s1*s4*s5 + c1*(c5*s23 + c23*c4*s5);

   J[4][0] = 0;
   J[4][1] = c1;
   J[4][2] = c1;
   J[4][3] = s1*s23;
   J[4][4] = c1*c4 - c23*s1*s4;
   J[4][5] = c5*s1*s23 + (c23*c4*s1 + c1*s4)*s5;

   J[5][0] = 1;
   J[5][1] = 0;
   J[5][2] = 0;
   J[5][3] = c23;
   J[5][4] = s23*s4;
   J[5][5] = c23*c5 - c4*s23*s5;


   // ****************************************************************
   // Derivative of Jacobian with respect to q1...q6.  Used to find dJ.
   // ****************************************************************

   static SAIMatrix6 dJ1;  // dJ/dq1

   dJ1[0][0] = -c1*(L2*c2 + (L3+L6*c5)*s23 + L6*c23*c4*s5) + s1*(L1+L6*s4*s5);
   dJ1[0][1] = -s1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   dJ1[0][2] = -s1*(c23*(L3 + L6*c5) - L6*c4*s23*s5);
   dJ1[0][3] = -L6*(c1*c4 - c23*s1*s4)*s5;
   dJ1[0][4] = -L6*(c1*c5*s4 - s1*(-c23*c4*c5 + s23*s5));
   dJ1[0][5] = 0;

   dJ1[1][0] = -s1*(L2*c2 + (L3+L6*c5)*s23 + L6*c23*c4*s5) - c1*(L1+L6*s4*s5);
   dJ1[1][1] = c1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   dJ1[1][2] = c1*(c23*(L3 + L6*c5) - L6*c4*s23*s5);
   dJ1[1][3] = L6*(-c4*s1 - c1*c23*s4)*s5;
   dJ1[1][4] = L6*(c1*c23*c4*c5 - c5*s1*s4 - c1*s23*s5);
   dJ1[1][5] = 0;

   dJ1[2][0] = 0;
   dJ1[2][1] = 0;
   dJ1[2][2] = 0;
   dJ1[2][3] = 0;
   dJ1[2][4] = 0;
   dJ1[2][5] = 0;

   dJ1[3][0] = 0;
   dJ1[3][1] = -c1;
   dJ1[3][2] = -c1;
   dJ1[3][3] = -s1*s23;
   dJ1[3][4] = -c1*c4 + c23*s1*s4;
   dJ1[3][5] = -c1*s4*s5 - s1*(c5*s23 + c23*c4*s5);

   dJ1[4][0] = 0;
   dJ1[4][1] = -s1;
   dJ1[4][2] = -s1;
   dJ1[4][3] = c1*s23;
   dJ1[4][4] = -c4*s1 - c1*c23*s4;
   dJ1[4][5] = c1*c5*s23 + (c1*c23*c4 - s1*s4)*s5;

   dJ1[5][0] = 0;
   dJ1[5][1] = 0;
   dJ1[5][2] = 0;
   dJ1[5][3] = 0;
   dJ1[5][4] = 0;
   dJ1[5][5] = 0;

   static SAIMatrix6 dJ2; // ddJ/dq2

   dJ2[0][0] = -s1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   dJ2[0][1] = -c1*(L2*c2 + (L3 + L6*c5)*s23 + L6*c23*c4*s5);
   dJ2[0][2] = c1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ2[0][3] = L6*c1*s23*s4*s5;
   dJ2[0][4] = -L6*c1*(c4*c5*s23 + c23*s5);
   dJ2[0][5] = 0;

   dJ2[1][0] = c1*(c23*(L3 + L6*c5) - L2*s2 - L6*c4*s23*s5);
   dJ2[1][1] = -s1*(L2*c2 + (L3 + L6*c5)*s23 + L6*c23*c4*s5);
   dJ2[1][2] = s1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ2[1][3] = L6*s1*s23*s4*s5;
   dJ2[1][4] = -L6*s1*(c4*c5*s23 + c23*s5);
   dJ2[1][5] = 0;

   dJ2[2][0] = 0;
   dJ2[2][1] = -c23*(L3 + L6*c5) + L2*s2 + L6*c4*s23*s5;
   dJ2[2][2] = -c23*(L3 + L6*c5) + L6*c4*s23*s5;
   dJ2[2][3] = L6*c23*s4*s5;
   dJ2[2][4] = -L6*c23*c4*c5 + L6*s23*s5;
   dJ2[2][5] = 0;

   dJ2[3][0] = 0;
   dJ2[3][1] = 0;
   dJ2[3][2] = 0;
   dJ2[3][3] = c1*c23;
   dJ2[3][4] = c1*s23*s4;
   dJ2[3][5] = c1*(c23*c5 - c4*s23*s5);

   dJ2[4][0] = 0;
   dJ2[4][1] = 0;
   dJ2[4][2] = 0;
   dJ2[4][3] = c23*s1;
   dJ2[4][4] = s1*s23*s4;
   dJ2[4][5] = s1*(c23*c5 - c4*s23*s5);

   dJ2[5][0] = 0;
   dJ2[5][1] = 0;
   dJ2[5][2] = 0;
   dJ2[5][3] = -s23;
   dJ2[5][4] = c23*s4;
   dJ2[5][5] = -c5*s23 - c23*c4*s5;

   static SAIMatrix6 dJ3; // ddJ/dq3

   dJ3[0][0] = s1*(-c23*(L3 + L6*c5) + L6*c4*s23*s5);
   dJ3[0][1] = c1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ3[0][2] = c1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ3[0][3] = L6*c1*s23*s4*s5;
   dJ3[0][4] = -L6*c1*(c4*c5*s23 + c23*s5);
   dJ3[0][5] = 0;

   dJ3[1][0] = c1*(c23*(L3 + L6*c5) - L6*c4*s23*s5);
   dJ3[1][1] = s1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ3[1][2] = s1*(-(L3 + L6*c5)*s23 - L6*c23*c4*s5);
   dJ3[1][3] = L6*s1*s23*s4*s5;
   dJ3[1][4] = -L6*s1*(c4*c5*s23 + c23*s5);
   dJ3[1][5] = 0;

   dJ3[2][0] = 0;
   dJ3[2][1] = -c23*(L3 + L6*c5) + L6*c4*s23*s5;
   dJ3[2][2] = -c23*(L3 + L6*c5) + L6*c4*s23*s5;
   dJ3[2][3] = L6*c23*s4*s5;
   dJ3[2][4] = -L6*c23*c4*c5 + L6*s23*s5;
   dJ3[2][5] = 0;

   dJ3[3][0] = 0;
   dJ3[3][1] = 0;
   dJ3[3][2] = 0;
   dJ3[3][3] = c1*c23;
   dJ3[3][4] = c1*s23*s4;
   dJ3[3][5] = c1*(c23*c5 - c4*s23*s5);

   dJ3[4][0] = 0;
   dJ3[4][1] = 0;
   dJ3[4][2] = 0;
   dJ3[4][3] = c23*s1;
   dJ3[4][4] = s1*s23*s4;
   dJ3[4][5] = s1*(c23*c5 - c4*s23*s5);

   dJ3[5][0] = 0;
   dJ3[5][1] = 0;
   dJ3[5][2] = 0;
   dJ3[5][3] = -s23;
   dJ3[5][4] = c23*s4;
   dJ3[5][5] = -c5*s23 - c23*c4*s5;

   static SAIMatrix6 dJ4; // ddJ/dq4

   dJ4[0][0] = L6*(-c1*c4 + c23*s1*s4)*s5;
   dJ4[0][1] = L6*c1*s23*s4*s5;
   dJ4[0][2] = L6*c1*s23*s4*s5;
   dJ4[0][3] = L6*(-c1*c23*c4 + s1*s4)*s5;
   dJ4[0][4] = -L6*c5*(c4*s1 + c1*c23*s4);
   dJ4[0][5] = 0;

   dJ4[1][0] = -L6*(c4*s1 + c1*c23*s4)*s5;
   dJ4[1][1] = L6*s1*s23*s4*s5;
   dJ4[1][2] = L6*s1*s23*s4*s5;
   dJ4[1][3] = -L6*(c23*c4*s1 + c1*s4)*s5;
   dJ4[1][4] = L6*c5*(c1*c4 - c23*s1*s4);
   dJ4[1][5] = 0;

   dJ4[2][0] = 0;
   dJ4[2][1] = L6*c23*s4*s5;
   dJ4[2][2] = L6*c23*s4*s5;
   dJ4[2][3] = L6*c4*s23*s5;
   dJ4[2][4] = L6*c5*s23*s4;
   dJ4[2][5] = 0;

   dJ4[3][0] = 0;
   dJ4[3][1] = 0;
   dJ4[3][2] = 0;
   dJ4[3][3] = 0;
   dJ4[3][4] = -c1*c23*c4 + s1*s4;
   dJ4[3][5] = -(c4*s1 + c1*c23*s4)*s5;

   dJ4[4][0] = 0;
   dJ4[4][1] = 0;
   dJ4[4][2] = 0;
   dJ4[4][3] = 0;
   dJ4[4][4] = -c23*c4*s1 - c1*s4;
   dJ4[4][5] = (c1*c4 - c23*s1*s4)*s5;

   dJ4[5][0] = 0;
   dJ4[5][1] = 0;
   dJ4[5][2] = 0;
   dJ4[5][3] = 0;
   dJ4[5][4] = c4*s23;
   dJ4[5][5] = s23*s4*s5;

   static SAIMatrix6 dJ5; // ddJ/dq5

   dJ5[0][0] = -L6*c5*(c23*c4*s1 + c1*s4) + L6*s1*s23*s5;
   dJ5[0][1] = -L6*c1*(c4*c5*s23 + c23*s5);
   dJ5[0][2] = -L6*c1*(c4*c5*s23 + c23*s5);
   dJ5[0][3] = -L6*c5*(c4*s1 + c1*c23*s4);
   dJ5[0][4] = L6*(s1*s4*s5 - c1*(c5*s23 + c23*c4*s5));
   dJ5[0][5] = 0;

   dJ5[1][0] = -L6*(c5*s1*s4 + c1*(-c23*c4*c5 + s23*s5));
   dJ5[1][1] = -L6*s1*(c4*c5*s23 + c23*s5);
   dJ5[1][2] = -L6*s1*(c4*c5*s23 + c23*s5);
   dJ5[1][3] = L6*c5*(c1*c4 - c23*s1*s4);
   dJ5[1][4] = -L6*(c5*s1*s23 + (c23*c4*s1 + c1*s4)*s5);
   dJ5[1][5] = 0;

   dJ5[2][0] = 0;
   dJ5[2][1] = -L6*c23*c4*c5 + L6*s23*s5;
   dJ5[2][2] = -L6*c23*c4*c5 + L6*s23*s5;
   dJ5[2][3] = L6*c5*s23*s4;
   dJ5[2][4] = -L6*c23*c5 + L6*c4*s23*s5;
   dJ5[2][5] = 0;

   dJ5[3][0] = 0;
   dJ5[3][1] = 0;
   dJ5[3][2] = 0;
   dJ5[3][3] = 0;
   dJ5[3][4] = 0;
   dJ5[3][5] = -c5*s1*s4 + c1*(c23*c4*c5 - s23*s5);

   dJ5[4][0] = 0;
   dJ5[4][1] = 0;
   dJ5[4][2] = 0;
   dJ5[4][3] = 0;
   dJ5[4][4] = 0;
   dJ5[4][5] = c23*c4*c5*s1 + c1*c5*s4 - s1*s23*s5;

   dJ5[5][0] = 0;
   dJ5[5][1] = 0;
   dJ5[5][2] = 0;
   dJ5[5][3] = 0;
   dJ5[5][4] = 0;
   dJ5[5][5] = -c4*c5*s23 - c23*s5;

   static SAIMatrix6 dJ6; // ddJ/dq6

   dJ6[0][0] = 0;
   dJ6[0][1] = 0;
   dJ6[0][2] = 0;
   dJ6[0][3] = 0;
   dJ6[0][4] = 0;
   dJ6[0][5] = 0;

   dJ6[1][0] = 0;
   dJ6[1][1] = 0;
   dJ6[1][2] = 0;
   dJ6[1][3] = 0;
   dJ6[1][4] = 0;
   dJ6[1][5] = 0;

   dJ6[2][0] = 0;
   dJ6[2][1] = 0;
   dJ6[2][2] = 0;
   dJ6[2][3] = 0;
   dJ6[2][4] = 0;
   dJ6[2][5] = 0;

   dJ6[3][0] = 0;
   dJ6[3][1] = 0;
   dJ6[3][2] = 0;
   dJ6[3][3] = 0;
   dJ6[3][4] = 0;
   dJ6[3][5] = 0;

   dJ6[4][0] = 0;
   dJ6[4][1] = 0;
   dJ6[4][2] = 0;
   dJ6[4][3] = 0;
   dJ6[4][4] = 0;
   dJ6[4][5] = 0;

   dJ6[5][0] = 0;
   dJ6[5][1] = 0;
   dJ6[5][2] = 0;
   dJ6[5][3] = 0;
   dJ6[5][4] = 0;
   dJ6[5][5] = 0;

   // ****************************************************************
   // dJ matrix (derivative of Jacobian with respect to time)
   // ****************************************************************

   dJ.setSize( 6, 6 );
   dJ = dJ1 * dq1 + dJ2 * dq2 + dJ3 * dq3 + dJ4 * dq4 + dJ5 * dq5 + dJ6 * dq6;

   // ****************************************************************
   // A matrix
   // ****************************************************************

   A.setSize( 6, 6 );

   A[0][0] = (Izz1 + L1*L1*(M2 + M3 + M4 + M5 + M6) + M1*R1*R1 + (M2*R2*R2)/2 +
              L2*L2*(M3 + M4 + M5 + M6)*c2*c2 + (M2*R2*R2*c22)/2 +
              s23*s23*(L3*L3*(M4 + M5 + M6) + M3*R3*R3 +
                       M6*R6*c5*(2*L3 + R6*c5) + Izz5*s4*s4) +
              (c4*(L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5)*s2323 +
               2*L1*M6*R6*s4)*s5 +
              (Izz6*c4*c4*s23*s23 + M6*R6*R6*s4*s4)*s5*s5 +
              2*L2*c2*((L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5)*s23 +
                       M6*R6*c23*c4*s5) +
              c23*c23*(Izz4 + Izz6*c5*c5 + M6*R6*R6*c4*c4*s5*s5));

   A[0][0] = (Izz1 + L1*L1*(M2 + M3 + M4 + M5 + M6) + M1*R1*R1 + M2*R2*R2/2 +
              L2*L2*(M3 + M4 + M5 + M6)*c2*c2 + 0.5*M2*R2*R2*c22 +
              s23*s23*(L3*L3*(M4 + M5 + M6) + M3*R3*R3 +
                       M6*R6*c5*(2*L3 + R6*c5) + Izz5*s4*s4) +
              (c4*(L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5)*(2*c23*s23) +
               2*L1*M6*R6*s4)*s5 +
              (Izz6*c4*c4*s23*s23 + M6*R6*R6*s4*s4)*s5*s5 +
              2*L2*c2*((L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5)*s23 +
                       M6*R6*c23*c4*s5) +
              c23*c23*(Izz4 + Izz6*c5*c5 + M6*R6*R6*c4*c4*s5*s5));
   A[0][1] = (s2*(L1*L2*(M3 + M4 + M5 + M6) + L1*M2*R2 + L2*M6*R6*s4*s5) -
              c23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
                   (L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5) +
              c4*s23*(L1*M6*R6*s5 + s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5)));
   A[0][2] = (-c23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
                    (L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5) +
              c4*s23*(L1*M6*R6*s5 + s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5)));
   A[0][3] = (c4*(L2*M6*R6*c2 + (L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5)*s23)*s5 +
              0.5*c23*(2*Izz4 + Izz6 + M6*R6*R6 + (Izz6 - M6*R6*R6)*c55 +
                       2*L1*M6*R6*s4*s5));
   A[0][4] = (Izz5*s23*s4 + M6*R6*(-L1*c23*c4*c5 + L2*c2*c5*s4 +
                                   s23*((R6 + L3*c5)*s4 + L1*s5)));
   A[0][5] = Izz6*(c23*c5 - c4*s23*s5);

   A[1][0] = A[0][1];
   A[1][1] = 0.25*(4*Izz2 + 4*Izz3 + 2*Izz5 + Izz6 +
                   4*(L3*L3*(M4 + M5 + M6) + L2*L2*(M3 + M4 + M5 + M6) +
                      M2*R2*R2 + M3*R3*R3) + 3*M6*R6*R6 +
                   ((2*Izz5 - Izz6 + M6*R6*R6))*c44 + 8*L2*L3*M4*s3 +
                   8*L2*L3*M5*s3 + 8*L2*L3*M6*s3 + 8*L2*M3*R3*s3 +
                   8*M6*R6*c5*((L3 + L2*s3)) - 2*Izz6*c55*s4*s4 +
                   2*M6*R6*R6*c55*s4*s4 + 8*L2*M6*R6*c3*c4*s5);
   A[1][2] = 0.25*((4*Izz3 + 2*Izz5 + Izz6 + 4*L3*L3*((M4 + M5 + M6)) +
                    4*M3*R3*R3 + 3*M6*R6*R6 + (2*Izz5 - Izz6 + M6*R6*R6)*c44 +
                    4*L2*L3*M4*s3 + 4*L2*L3*M5*s3 + 4*L2*L3*M6*s3 +
                    4*L2*M3*R3*s3 + 4*M6*R6*c5*((2*L3 + L2*s3)) -
                    2*Izz6*c55*s4*s4 + 2*M6*R6*R6*c55*s4*s4 +
                    4*L2*M6*R6*c3*c4*s5));
   A[1][3] = (-(((((-Izz6) + M6*R6*R6))*c5 + M6*R6*((L3 + L2*s3)))))*s4*s5;
   A[1][4] = Izz5*c4 + M6*R6*((c4*((R6 + c5*((L3 + L2*s3)))) + L2*c3*s5));
   A[1][5] = Izz6*s4*s5;

   A[2][0] = A[0][2];
   A[2][1] = A[1][2];
   A[2][2] = 0.25*(4*Izz3 + 2*Izz5 + Izz6 + 4*L3*L3*((M4 + M5 + M6)) +
                   4*M3*R3*R3 + 3*M6*R6*R6 + (2*Izz5 - Izz6 + M6*R6*R6)*c44 +
                   8*L3*M6*R6*c5 - 2*((Izz6 - M6*R6*R6))*c55*s4*s4);
   A[2][3] = -(L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5;
   A[2][4] = c4*((Izz5 + M6*R6*((R6 + L3*c5))));
   A[2][5] = Izz6*s4*s5;

   A[3][0] = A[0][3];
   A[3][1] = A[1][3];
   A[3][2] = A[2][3];
   A[3][3] = Izz4 + 0.5*((Izz6 + M6*R6*R6 + ((Izz6 - M6*R6*R6))*c55));
   A[3][4] = 0;
   A[3][5] = Izz6*c5;

   A[4][0] = A[0][4];
   A[4][1] = A[1][4];
   A[4][2] = A[2][4];
   A[4][3] = A[3][4];
   A[4][4] = Izz5 + M6*R6*R6;
   A[4][5] = 0;

   A[5][0] = A[0][5];
   A[5][1] = A[1][5];
   A[5][2] = A[2][5];
   A[5][3] = A[3][5];
   A[5][4] = A[4][5];
   A[5][5] = Izz6;

   // ****************************************************************
   // B vector
   // ****************************************************************

   Float a112 = -(L2*L2*(M3 + M4 + M5 + M6)*s22) - M2*R2*R2*s22 +
      2*c23* s23*(L3*L3*(M4 + M5 + M6) + M3*R3*R3 + M6*R6*c5*(2*L3 + R6*c5) +
      Izz5*s4*s4) + 2*c2323*c4*(L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s5 +
      Izz6*c4*c4*s2323*s5*s5 - 2*L2*s2*((L3*(M4 + M5 + M6) + M3*R3 +
      M6*R6*c5)*s23 + M6*R6*c23*c4*s5) + 2*L2*c2*(c23*(L3*(M4 + M5 + M6) +
      M3*R3 + M6*R6*c5) - M6*R6*c4*s23*s5) - 2*c23*s23*(Izz4 + Izz6*c5*c5 +
      M6*R6*R6*c4*c4*s5*s5);
   Float a113 = 2*(c2323* c4*(L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5)* s5 +
      L2*c2*(c23*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) - M6*R6*c4*s23*s5) +
      c23*s23*(-Izz4 + L3*L3*(M4 + M5 + M6) + M3*R3*R3 + c5*(2*L3*M6*R6 -
      Izz6*c5 + M6*R6*R6*c5) + Izz5*s4*s4 + (Izz6 - M6*R6*R6)* c4*c4*s5*s5));
   Float a114 = (2*L1*M6*R6* c4 - (2*L2*M6*R6*c2* c23 + (L3*M6*R6 -
      Izz6*c5 + M6*R6*R6*c5)*s2323)*s4)*s5 + s23*s23*(-2*Izz6*c4*s4*s5*s5 +
      s44*(Izz5 + M6*R6*R6*s5*s5));
   Float a115 = c4*(L3*M6*R6*c5 + (-Izz6 + M6*R6*R6)*c55)*s2323 +
      2*L2*M6*R6*c2*(c23*c4*c5 - s23*s5) + 2*M6*R6*(L1*c5*s4 - L3*s23*s23*s5) -
      ((Izz6 - M6*R6*R6)*(1 + 3*c2323 - 2*c44*s23*s23)*s55)/4;
   Float a116 = 0;

   Float a122 = c2*(L1*L2*(M3 + M4 + M5 + M6) + L1*M2*R2 +
      L2*M6*R6*s4*s5) + s23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
      (L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5) + c23*c4*(L1*M6*R6*s5 +
      s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5));
   Float a123 = s23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
      (L3*M6*R6 + (-Izz6 + M6*R6*R6)* c5)*s4*s5) + c23*c4*(L1*M6*R6*s5 +
      s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5));
   Float a124 = c4*(-(c23*(L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)) +
      L2*M6*R6*s2)*s5 + s23*(-(L1*M6*R6*s4*s5) +
      c44*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5));
   Float a125 = L2*M6*R6*c5*s2*s4 + c23*(-(L3*M6*R6*c5*s4) +
      (Izz6 - M6*R6*R6)* c55*s4 + L1*M6*R6*s5) + c4*c5*s23*(L1*M6*R6 -
      2*(Izz6 - M6*R6*R6)*s4*s5);
   Float a126 = 0;

   Float a132 = s23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
      (L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5) + c23*c4*(L1*M6*R6*s5 +
      s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5));
   Float a133 = s23*(L1*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) +
      (L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s4*s5) + c23*c4*(L1*M6*R6*s5 +
      s4*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5));
   Float a134 = -((c23*c4*(L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5) +
      L1*M6*R6*s23*s4)*s5) + c44*s23*(Izz5 + (-Izz6 + M6*R6*R6)*s5*s5);
   Float a135 = c23*(-(L3*M6*R6*c5*s4) + (Izz6 - M6*R6*R6)*c55*s4 +
      L1*M6*R6*s5) + c4*c5*s23*(L1*M6*R6 - 2*(Izz6 - M6*R6*R6)*s4*s5);
   Float a136 = 0;

   Float a142 = c4*(c23*(L3*M6*R6 - Izz6*c5 + M6*R6*R6*c5) -
      L2*M6*R6*s2)*s5 - (s23*(2*Izz4 + Izz6 + M6*R6*R6 + (Izz6 -
      M6*R6*R6)*c55 + 2*L1*M6*R6*s4*s5))/2;
   Float a143 = c23*c4*(L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s5 -
      (s23*(2*Izz4 + Izz6 + M6*R6*R6 + (Izz6 - M6*R6*R6)*c55 +
      2*L1*M6*R6*s4*s5))/2;
   Float a144 = (L1*M6*R6*c23*c4 - (L2*M6*R6*c2 + (L3*M6*R6 - Izz6*c5 +
      M6*R6*R6*c5)* s23)*s4)*s5;
   Float a145 = L2*M6*R6*c2*c4*c5 + c4*(L3*M6*R6*c5 + (-Izz6 +
      M6*R6*R6)*c55)*s23 + c23*c5*(-2*Izz6*s5 + M6*R6*(L1*s4 + 2*R6*s5));
   Float a146 = 0;

   Float a152 = Izz5*c23*s4 + M6*R6*(L1*c4*c5*s23 - L2*c5*s2*s4 +
      c23*((R6 + L3*c5)*s4 + L1*s5));
   Float a153 = L1*M6*R6*c4*c5*s23 + c23*((Izz5 + M6*R6*R6 +
      L3*M6*R6*c5)*s4 + L1*M6*R6*s5);
   Float a154 = L2*M6*R6*c2*c4*c5 + c4*(Izz5 + M6*R6*R6 +
      L3*M6*R6*c5)*s23 + L1*M6*R6*c23*c5*s4;
   Float a155 = M6*R6*(L1*c5*s23 + (L1*c23*c4 - (L2*c2 + L3*s23)*s4)*s5);
   Float a156 = 0;

   Float a162 = -(Izz6*(c5*s23 + c23*c4*s5));
   Float a163 = -(Izz6*(c5*s23 + c23*c4*s5));
   Float a164 = Izz6*s23*s4*s5;
   Float a165 = -(Izz6*(c4*c5*s23 + c23*s5));
   Float a166 = 0;

   Float a223 = 2*L2*(c3*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) -
      M6*R6*c4*s3*s5);
   Float a224 = ((-2*Izz5 + Izz6 - M6*R6*R6 + (-Izz6 +
      M6*R6*R6)*c55)*s44 - 4*L2*M6*R6*c3*s4*s5)/2;
   Float a225 = 2*L2*M6*R6*c3*c4*c5 - 2*M6*R6*(L3 + L2*s3)*s5 +
      (Izz6 - M6*R6*R6)*s4*s4*s55;
   Float a226 = 0;

   Float a233 = L2*(c3*(L3*(M4 + M5 + M6) + M3*R3 + M6*R6*c5) -
      M6*R6*c4*s3*s5);
   Float a234 = ((-2*Izz5 + Izz6 - M6*R6*R6 +
      (-Izz6 + M6*R6*R6)*c55)*s44 - 2*L2*M6*R6*c3*s4*s5)/2;
   Float a235 = L2*M6*R6*c3*c4*c5 - M6*R6*(2*L3 + L2*s3)*s5 +
      (Izz6 - M6*R6*R6)*s4*s4*s55;
   Float a236 = 0;

   Float a243 = -(L2*M6*R6*c3*s4*s5);
   Float a244 = -(c4*((-Izz6 + M6*R6*R6)*c5 + M6*R6*(L3 + L2*s3))*s5);
   Float a245 = -(((-Izz6 + M6*R6*R6)*c55 + M6*R6*c5*(L3 + L2*s3))*s4);
   Float a246 = 0;

   Float a253 = L2*M6*R6*(c3*c4*c5 - s3*s5);
   Float a254 = (-Izz5 - M6*R6*(R6 + c5*(L3 + L2*s3)))*s4;
   Float a255 = M6*R6*(L2*c3*c5 - c4*(L3 + L2*s3)*s5);
   Float a256 = 0;

   Float a263 = 0;
   Float a264 = Izz6*c4*s5;
   Float a265 = Izz6*c5*s4;
   Float a266 = 0;

   Float a334 = -((2*Izz5 - Izz6 + M6*R6*R6 + (Izz6 -
      M6*R6*R6)*c55)*s44)/2;
   Float a335 = -2*L3*M6*R6*s5 + (Izz6 - M6*R6*R6)*s4*s4*s55;
   Float a336 = 0;

   Float a344 = -(c4*(L3*M6*R6 + (-Izz6 + M6*R6*R6)*c5)*s5);
   Float a345 = -((L3*M6*R6*c5 + (-Izz6 + M6*R6*R6)*c55)*s4);
   Float a346 = 0;

   Float a354 = -((Izz5 + M6*R6*(R6 + L3*c5))*s4);
   Float a355 = -(L3*M6*R6*c4*s5);
   Float a356 = 0;

   Float a364 = Izz6*c4*s5;
   Float a365 = Izz6*c5*s4;
   Float a366 = 0;

   Float a445 = (-Izz6 + M6*R6*R6)*s55;
   Float a446 = 0;

   Float a455 = 0;
   Float a456 = 0;

   Float a465 = -(Izz6*s5);
   Float a466 = 0;

   Float a556 = 0;

   Float a566 = 0;

   B.setSize( 6 );
   B[0] =
      dq1 * (a112*dq2 + a113*dq3 + a114*dq4 + a115*dq5 + a116*dq6) +
      dq2 * (a122*dq2 + (a123 + a132)*dq3 + (a124 + a142)*dq4 +
             (a125 + a152)*dq5 + (a126 + a162)*dq6) +
      dq3 * (a133*dq3 + (a134 + a143)*dq4 + (a135 + a153)*dq5 +
             (a136 + a163)*dq6) +
      dq4 * (a144*dq4 + (a145 + a154)*dq5 + (a146 + a164)*dq6) +
      dq5 * (a155*dq5 + (a156 + a165)*dq6) +
      dq6 * (a166*dq6);

   B[1] =
      dq1 * ((-a112/2)*dq1 + (a123 - a132)*dq3 + (a124 - a142)*dq4 +
             (a125 - a152)*dq5 + (a126 - a162)*dq6) +
      dq2 * (a223*dq3 + a224*dq4 + a225*dq5 + a226*dq6) +
      dq3 * (a233*dq3 + (a234 + a243)*dq4 + (a235 + a253)*dq5 + 
             (a236 + a263)*dq6) +
      dq4 * (a244*dq4 + (a245 + a254)*dq5 + (a246 + a264)*dq6) +
      dq5 * (a255*dq5 + (a256 + a265)*dq6) +
      dq6 * (a266*dq6);

   B[2] =
      dq1 * ((-a113/2)*dq1 + (-a123 + a132)*dq2 + (a134 - a143)*dq4 +
             (a135 - a153)*dq5 + (a136 - a163)*dq6) +
      dq2 * ((-a223/2)*dq2 + (a234 - a243)*dq4 + (a235 - a253)*dq5 +
             (a236 - a263)*dq6) +
      dq3 * (a334*dq4 + a335*dq5 + a336*dq6) +
      dq4 * (a344*dq4 + (a345 + a354)*dq5 + (a346 + a364)*dq6) +
      dq5 * (a355*dq5 + (a356 + a365)*dq6) +
      dq6 * (a366*dq6);

   B[3] =
      dq1 * ((-a114/2)*dq1 + (-a124 + a142)*dq2 + (-a134 + a143)*dq3 +
             (a145 - a154)*dq5 + (a146 - a164)*dq6) +
      dq2 * ((-a224/2)*dq2 + (-a234 + a243)*dq3 + (a245 - a254)*dq5 +
             (a246 - a264)*dq6) +
      dq3 * ((-a334/2)*dq3 + (a345 - a354)*dq5 + (a346 - a364)*dq6) +
      dq4 * (a445*dq5 + a446*dq6) +
      dq5 * (a455*dq5 + (a456 + a465)*dq6) +
      dq6 * (a466*dq6);

   B[4] =
      dq1 * ((-a115/2)*dq1 + (-a125 + a152)*dq2 + (-a135 + a153)*dq3 +
             (-a145 + a154)*dq4 + (a156 - a165)*dq6) +
      dq2 * ((-a225/2)*dq2 + (-a235 + a253)*dq3 + (-a245 + a254)*dq4 +
             (a256 - a265)*dq6) +
      dq3 * ((-a335/2)*dq3 + (-a345 + a354)*dq4 + (a356 - a365)*dq6) +
      dq4 * ((-a445/2)*dq4 + (a456 - a465)*dq6) +
      dq5 * (a556*dq6) +
      dq6 * (a566*dq6);

   B[5] =
      dq1 * ((-a116/2)*dq1 + (-a126 + a162)*dq2 + (-a136 + a163)*dq3 + 
             (-a146 + a164)*dq4 + (-a156 + a165)*dq5) +
      dq2 * ((-a226/2)*dq2 + (-a236 + a263)*dq3 + (-a246 + a264)*dq4 +
             (-a256 + a265)*dq5) +
      dq3 * ((-a336/2)*dq3 + (-a346 + a364)*dq4 + (-a356 + a365)*dq5) +
      dq4 * ((-a446/2)*dq4 + (-a456 + a465)*dq5) +
      dq5 * ((-a556/2)*dq5) +
      dq6 * (0);

   // ****************************************************************
   // G vector
   // ****************************************************************

   G.setSize( 6 );

   G[0] = 0;
   G[1] = -R2*c2*M2 - (L2*c2 + R3*s23)*M3 - (L2*c2 + L3*s23)*(M4 + M5 + M6) -
          R6*(c5*s23 + c23*c4*s5)*M6;
   G[2] = -R3*s23*M3 - L3*s23*(M4 + M5 + M6) - R6*(c5*s23 - c23*c4*s5)*M6;
   G[3] = R6*s23*s4*s5*M6;
   G[4] = -R6*(c4*c5*s23 + c23*s5)*M6;
   G[5] = 0;

   G *= -GRAVITY;
}

void getPumaSingularities( Float sbound, const SAIVector& q,
                           bool& headLock, bool& elbowLock, bool& wristLock )
{
   Float q2  = q[1];
   Float q3  = q[2];
   Float q5  = q[4];
   Float c2  = cos( q2 );
   Float s23 = sin( q2 + q3 );

   headLock  = ( fabs( L2*c2 + L3*s23 ) < sbound );
   elbowLock = ( fabs( q3 - M_PI/2 ) < sbound );
   wristLock = ( fabs( q5 ) < sbound );
}
