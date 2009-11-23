/* Copyright (c) 2005 Arachi, Inc. and Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _taoABDynamics_h
#define _taoABDynamics_h

#include "taoTypes.h"

class taoDNode;
class deMatrix3;
class deMatrix6;
class deVector3;
class deVector6;
class deFrame;
class taoABDynamicsData;
class taoABDynamicsData2;

//#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 *	\brief		Articulated body dynamics class
 *	\ingroup	taoDynamics
 *
 *	This class provides inverse and forward dynamics for articulated body
 */
class taoABDynamics
{
public:
	//! updates local transforms in the subtree with \a root
	static void updateLocalXTreeOut(taoDNode* root);
	//! resets inertias in the subtree with \a root
	static void resetInertiaTreeOut(taoDNode* root);
	//! resets flags in the subtree with \a root
	static void resetFlagTreeOut(taoDNode* root);
	//! computes forward dynamics of the subtree with \a root under \a gravity
	/*!
	 *	\pre	q, dq, tau should be given
	 *	\post	ddq is acceleration
	 */
	static void forwardDynamics(taoDNode* root, const deVector3* gravity);
	//! computes inverse dynamics of the subtree with \a root under \a gravity
	/*!
	 *	\pre	q, dq, ddq should be given, Fext
	 *	\post	tau is computed torque
	 */
	static void inverseDynamics(taoDNode* root, const deVector3* gravity);

	//! computes velocity changes given impulse
	/*!
	 *	\param	point	expressed in local frame
	 *
	 *	\remarks	use this methods after forwardDynamicsConifgInit
	 *	\pre	tau (impulse) is given only to the \a contact at local \a point with \a impulse 
	 *	\post	ddq (velocity change) are computed for all nodes, V, dQ are also changed
	 */
	static void forwardDynamicsImpulse(taoDNode* contact, const deVector3* point, const deVector3* impulse, const deInt dist);
	//! computes global Jacobina matrices
	/*!
	 *	\pre	q
	 *	\post	Jg
	 */
	static void globalJacobianOut(taoDNode* root);
	//! computes inverse operational space inertia matrix for each node: diagonal terms: Omega_i_i
	/*!
	 *	\pre	S, Dinv, L
	 *	\post	Omega
	 */
	static void opSpaceInertiaMatrixOut(taoDNode* root, const deMatrix6* Oah = NULL);
	static void biasAccelerationOut(taoDNode* root, const deVector6* Hh = NULL);
	static void compute_Jg_Omega_H(taoDNode* root, const deMatrix6* Oah = NULL, const deVector6* Hh = NULL);
	static deFloat effectiveMassInv(taoDNode* node, const deVector3* point, const deVector3* dir);

	// ddXn is expressed in {n} for {n}  --> ddX = n ddX n
	static void opSpaceInvDynamics(taoDNode* opNode, deVector6* ddXn);
	// Fg is expressed in {o} for {n}
	static void add2Tau_JgT_F(taoDNode* node, const deVector6* Fg);

	//!	computes the potential energy of the subtree with \a root under \a gravity
	/*!
	 *	\return	the total potential energy
	 */
	static deFloat potentialEnergy(taoDNode* root, const deVector3* gravity);
	//!	computes the kinetic energy of the subtree with \a root under \a gravity
	/*!
	 *	\param	Vh	spatial velocity vector of \a root
	 *	\return	the total kinetic energy
	 */
	static deFloat kineticEnergy(taoDNode* root, const deVector6* Vh = NULL);
	//! resets inertia of \a node
	static void resetInertia(taoDNode* node);

private:
	static void _forwardDynamicsOutIn(taoDNode* root, taoABDynamicsData* datah, deVector6* Pah, const deVector6* Vh);
	static void _inverseDynamicsOutIn(taoDNode* root, taoABDynamicsData2* datah, deVector6* Fh, const deVector6* Vh, const deVector6* Ah);
	static void _accelerationTreeOut(taoDNode* root, const deVector6* Ah);
	static void _articulatedImpulsePathIn(taoDNode* contact, const deInt dist);
	static void _velocityDeltaTreeOut(taoDNode* root, const deVector6* dVh, const deVector6* Vh, const deInt dist);
	static void _plusEq_Jg_ddQ(taoDNode* node, deVector6* A);
};

//#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif // _taoABDynamics_h

