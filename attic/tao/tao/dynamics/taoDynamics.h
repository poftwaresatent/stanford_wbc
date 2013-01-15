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

#ifndef _taoDynamics_h
#define _taoDynamics_h

#include "taoTypes.h"

class taoDNode;
class deVector3;
class deVector6;

/*!
 *	\brief articulated body dynamics
 *	\ingroup taoDynamics
 *
 *	This class provides various dynamics computations for articulated body.
 */
class taoDynamics
{
public:
	static void initialize(taoDNode* root);
	/*!
	 *	\remarks	calls r->sync(r->frameHome())
	 */	
	static void reset(taoDNode* root);
	static void updateTransformation(taoDNode* root);

	static void integrate(taoDNode* root, deFloat dt);

	//! computes global Jacobina matrices
	/*!
	 *	\pre	q
	 *	\post	Jg
	 */
	static void globalJacobian(taoDNode* root);

	//! computes inverse dynamics of the subtree with \a root under \a gravity
	/*!
	 *	\pre	q, dq, ddq should be given, Fext
	 *	\post	tau is computed torque
	 */
	static void invDynamics(taoDNode* root, const deVector3* gravity);
	//! computes forward dynamics of the subtree with \a root under \a gravity
	/*!
	 *	\pre	q, dq, tau should be given
	 *	\post	ddq is acceleration
	 */
	static void fwdDynamics(taoDNode* root, const deVector3* gravity);


	//! computes Joint Space Inertia Matrix, \a A of size \a dof x \a dof
	/*! assuming current configuration/velocity
	* \remarks tau = A * ddq + b + g
	* \remarks ddq = Ainv * (tau - b - g)
	* \remarks ************* output
	* \remarks A = joint space inertia matrix
	* \remarks B = centrifugal / Coriolis force
	* \remarks G = gravity
	* \remarks Ainv = inverse of mass matrix
	* \remarks ************* Usage
	* \remarks {
	* \remarks		fwdDynamics(root, gravity);
	* \remarks		int dof = computeDOF(root);
	* \remarks		deFloat A[dof * dof]
	* \remarks		deFloat B[dof], G[dof]
	* \remarks		computeA(root, gravity, dof, A);
	* \remarks		computeB(root, gravity, dof, B);
	* \remarks		computeG(root, gravity, dof, G);
	* \remarks		computeAinv(root, gravity, dof, Ainv);
	* \remarks }
	*/
	static void computeA(taoDNode* root, const deInt dof, deFloat* A);
	//! computes Joint Space Inertia Matrix Inverse, \a Ainv (\a dof x \a dof)
	static void computeAinv(taoDNode* root, const deInt dof, deFloat* Ainv);
	//! computes Coriolis and centrifugal forces, \a B (\a dof x 1)
	static void computeB(taoDNode* root, const deInt dof, deFloat* B);
	//! computes gravitational forces, \a G (\a dof x 1) under \a gravity
	static void computeG(taoDNode* root, deVector3* gravity, const deInt dof, deFloat* G);

	//! compute the operational Space Inertia Matrix Inverse, \a Linv (\a row x \a row)
	/*!
	* \param J Jacobian matrix of the operational points (\a row x \a dof)
	* \param Ainv (\a dof x \a dof)
	* \param Linv (\a row x \a row) = J Ainv J^T
	*
	* \remarks effective mass in the direction of u can be found: m_u = 1 (u^T Linv u)
	*/
	static void computeOpSpaceInertiaMatrixInv(taoDNode* root, const deFloat* J, const deInt row, const deInt dof, const deFloat* Ainv, deFloat* Linv);

	//! find dof : degrees of freedom
	static int computeDOF(taoDNode* obj);

	//! computes velocity changes given impulse
	/*!
	 *	\param	point	expressed in local frame
	 *
	 *	\remarks	use this methods after forwardDynamicsConifgInit
	 *	\pre	tau (impulse) is given only to the \a contact at local \a point with \a impulse 
	 *	\post	ddq (velocity change) are computed for all nodes, V, dQ are also changed
	 */
	static void impulse(taoDNode* contactNode, const deVector3* contactPoint, const deVector3* impulseVector);
	static void impulseDist(taoDNode* contactNode, const deVector3* contactPoint, const deVector3* impulseVector);
	static void force(taoDNode* contactNode, const deVector3* contactPoint, const deVector3* forceVector);
	static void resetMass(taoDNode* node, const deFloat mass);

	static deFloat potentialEnergy(taoDNode* root, const deVector3* gravity);
	static deFloat kineticEnergy(taoDNode* root);

private:
	typedef enum {TAO_DDQ, TAO_DQ, TAO_TAU} flagType;
	static void _Read(taoDNode* root, deFloat* v, flagType type);
	static void _Write(taoDNode* root, deFloat* v, flagType type);
};

#endif // _taoDynamics_h
