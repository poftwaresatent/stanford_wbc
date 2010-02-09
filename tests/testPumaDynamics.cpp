/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file testPumaDynamics.cpp
   \author Roland Philippsen
*/

#include "puma/pumaDynamics.h"

int main(int argc, char ** argv)
{
  PrVector m_q(6);
  PrVector m_dq(6);
  PrMatrix m_J(6, 6);
  PrMatrix m_dJ(6, 6);
  PrMatrix m_A(6, 6);
  PrVector m_B(6);
  PrVector m_G(6);
  
  for (double qq(-0.1); qq <= 0.11; qq += 0.1) {
    for (int ii(0); ii < 6; ++ii) {
      m_q[ii] = qq;
      m_dq[ii] = 0;
    }
    getPumaDynamics(m_q, m_dq, m_J, m_dJ, m_A, m_B, m_G);
    printf("==================================================\n");
    m_q.display("q");
    m_dq.display("dq");
    m_J.display("J");
    m_dJ.display("dJ");
    m_A.display("A");
    m_B.display("B");
    m_G.display("G");
  }
}
