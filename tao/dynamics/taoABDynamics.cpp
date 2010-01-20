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

#include "taoABDynamics.h"
#include <tao/matrix/TaoDeMath.h>
#include "taoDNode.h"
#include "taoABNode.h"

#include <assert.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class taoABDynamicsData
{
public:
	deMatrix6 Ia;
	deVector3 WxV;
	deVector3 g;
};

class taoABDynamicsData2
{
public:
	deVector3 WxV;
	deVector3 g;
};

#endif // DOXYGEN_SHOULD_SKIP_THIS

void taoABDynamics::updateLocalXTreeOut(taoDNode* root)
{
	root->getABNode()->updateLocalX(*root->frameHome(), *root->frameLocal());

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		updateLocalXTreeOut(n);
}

void taoABDynamics::resetInertia(taoDNode* node)
{
	node->getABNode()->inertia(node->mass(), node->center(), node->inertia());
}

void taoABDynamics::resetInertiaTreeOut(taoDNode* root)
{
	resetInertia(root);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		resetInertiaTreeOut(n);
}

void taoABDynamics::resetFlagTreeOut(taoDNode* root)
{
	root->getABNode()->setFlag(0);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		resetFlagTreeOut(n);
}

void taoABDynamics::opSpaceInertiaMatrixOut(taoDNode* root, const deMatrix6* Oah)
{
	deMatrix6* Oa = root->getABNode()->Omega();

	root->getABNode()->osInertiaInv(*Oa, *Oah);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		opSpaceInertiaMatrixOut(n, Oa);
}

void taoABDynamics::globalJacobianOut(taoDNode* root)
{
	root->getABNode()->globalJacobian(*root->frameGlobal());

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		globalJacobianOut(n);
}

void taoABDynamics::biasAccelerationOut(taoDNode* root, const deVector6* Hh)
{
	deVector6* H = root->getABNode()->H();

	root->getABNode()->biasAcceleration(*H, *Hh);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		biasAccelerationOut(n, H);
}

// Oc = iXc^T Oi iXc  
// Oi = [ O11 O12 ; O12^T O22]
// iRc = I3 ---> orientation of {c} is the same as {i}
// iXc = [ R 0; rx R R ] = [ 1 0; rx 1 ]  ---> r is the point expressed in {i}
// iXc^T = [ 1 -rx; 0 1 ]
// 1/mu = uc^T Oc uc ---> mu is the effective mass in u dir
// uc = [ u ; 0 ] --> direction vector expressed in {i}
// 1/mu = [ u^T 0 ] [ 1 -rx; 0 1 ] Oi [ 1 0; rx 1 ] [ u ; 0 ]
// 1/mu = [ u^T  -u^T rx ] Oi [ u ; rx u]
//      = [ u^T  -u^T rx ] [ O11 O12 ; O12^T O22] [ u ; rx u]
//      = [ (u^T O11 - u^T rx O12^T ) (u^T O12 - u^T rx O22) ] [ u ; rx u]
//      = u^T O11 u - u^T rx O12^T u + u^T O12 rx u - u^T rx O22 rx u
//      = u^T O11 u + (rx u)^T O12^T u + u^T O12 (rx u) + (rx u)^T O22 (rx u)
//      = u^T O11 u + (rx u)^T O12^T u + ((rx u)^T O12^T u)^T + (rx u)^T O22 (rx u)
//      = u^T O11 u + 2*(rx u)^T O12^T u + (rx u)^T O22 (rx u)
//      = u^T O11 u + (rx u)^T [2*O12^T u + O22 (rx u)]
//      = u^T O11 u + (rx u)^T [2*O21 u + O22 (rx u)]
deFloat taoABDynamics::effectiveMassInv(taoDNode* node, const deVector3* point, const deVector3* dir)
{
#if 1
	deFloat muinv;
	deVector3 rxu, v, v1;

	v.multiply((*node->getABNode()->Omega())[0][0], *dir);
	muinv = dir->dot(v);

	v.multiply((*node->getABNode()->Omega())[1][0], *dir);
	v += v;

	rxu.crossMultiply(*point, *dir);

	v1.multiply((*node->getABNode()->Omega())[1][1], rxu);

	v += v1;

	muinv += rxu.dot(v);

	return muinv;
#elif 0
	deFloat muinv;
	deVector3 rxu, v, v1;
	deMatrix6 Omega;
	Omega.inverseSPD(*node->getABNode()->I());
	v.multiply(Omega[0][0], *dir);
	muinv = dir->dot(v);

	v.multiply(Omega[1][0], *dir);
	v += v;

	rxu.crossMultiply(*point, *dir);

	v1.multiply(Omega[1][1], rxu);

	v += v1;

	muinv += rxu.dot(v);

	return muinv;
#else
	deFloat mu;
	deVector3 v;

	v.multiply((*node->getABNode()->I())[0][0], *dir);
	mu = dir->dot(v);

	return 1 / mu;
#endif
}

void taoABDynamics::compute_Jg_Omega_H(taoDNode* root, const deMatrix6* Oah, const deVector6* Hh)
{
	deMatrix6* Oa = root->getABNode()->Omega();
	deVector6* H = root->getABNode()->H();

	root->getABNode()->globalJacobian(*root->frameGlobal());
	root->getABNode()->osInertiaInv(*Oa, *Oah);
	root->getABNode()->biasAcceleration(*H, *Hh);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		compute_Jg_Omega_H(n, Oa, H);
}

void taoABDynamics::_plusEq_Jg_ddQ(taoDNode* node, deVector6* A)
{
	if (!node->isRoot())
	{
		node->getABNode()->plusEq_Jg_ddQ(*A);
		_plusEq_Jg_ddQ(node->getDParent(), A);
	}
}

void taoABDynamics::add2Tau_JgT_F(taoDNode* node, const deVector6* Fg)
{
	if (!node->isRoot())
	{
		node->getABNode()->add2Tau_JgT_F(*Fg);
		add2Tau_JgT_F(node->getDParent(), Fg);
	}
}

// Tau = Je^T Fe + Tau_null
// Tau_null = A ddq_null + b + g
// Je = [ ... iXe^T Si ... ]
// Fe = Le (ddXe - He - Je ddq)
// Je^T Fe = Jn^T Fn
// Le^-1 = nXe^T On nXe
// Fe = nXe^-1 On^-1 nXe^-T (nXe^T ddXn - nXe^T Hn - nXe^T Jn ddq)
//    = nXe^-1 On^-1 (ddXn - Hn - Jn ddq)
// Je^T Fe = (nXe^T Jn)^T nXe^-1 On^-1 (ddXn - Hn - Jn ddq) = Jn^T On^-1 (ddXn - Hn - Jn ddq) = Jn^T Fn
//         
// Jn = oXn^T Jo
// Je^T Fe = Jn^T Fn = (oXn^T Jo)^T On^-1 (ddXn - Hn - oXn^T Jo ddq)
//                   = Jo^T oXn On^-1 (ddXn - Hn - oXn^T Jo ddq)
//         = Jo^T Fo
// Jo = [ ... iXo^T Si ... ] = [ ... oXi^-T Si ]
// Fo = oXn On^-1 (ddXn - Hn - oXn^T Jo ddq)
void taoABDynamics::opSpaceInvDynamics(taoDNode* opNode, deVector6* ddXn)
{
	// OpSpace
	// path only

	// ddX is expressed in {n} for {n}  --> ddX = o ddX n

	deTransform oXn;
	oXn.set(*opNode->frameGlobal());	// YYY

	*ddXn -= *opNode->getABNode()->H();

	deVector6 Fo, Fn;
	Fo.zero();
	_plusEq_Jg_ddQ(opNode, &Fo);

	Fn.xformT(oXn, Fo);

	*ddXn -= Fn;

	Fn.solveSPD(*opNode->getABNode()->Omega(), *ddXn);
	
	Fo.xform(oXn, Fn);

	// Tau = Tau_null
	// Tau += Jt F
	add2Tau_JgT_F(opNode, &Fo);
}

void taoABDynamics::forwardDynamics(taoDNode* root, const deVector3* gravity)
{
	taoABDynamicsData data;
	data.g = *gravity;

	_forwardDynamicsOutIn(root, &data, NULL, NULL);

	_accelerationTreeOut(root, NULL);
}

void taoABDynamics::inverseDynamics(taoDNode* root, const deVector3* gravity)
{
	taoABDynamicsData2 data;
	data.g = *gravity;

	_inverseDynamicsOutIn(root, &data, NULL, NULL, NULL);
}

void taoABDynamics::forwardDynamicsImpulse(taoDNode* contact, const deVector3* point, const deVector3* impulse, const deInt dist)
{
	assert(!contact->isRoot());

	contact->getABNode()->impulseInit(*point, *impulse);
	contact->getABNode()->setFlag(1);
	_articulatedImpulsePathIn(contact, dist);
}

void taoABDynamics::_articulatedImpulsePathIn(taoDNode* contact, const deInt dist)
{
	taoDNode *n = contact->getDParent();

	contact->getABNode()->abImpulse(*(n->getABNode()->Pa()), !n->isRoot());

	if (!n->isRoot())
	{
		n->getABNode()->setFlag(1);
		_articulatedImpulsePathIn(n, dist);
	}
	else
		_velocityDeltaTreeOut(n, NULL, NULL, dist);
}


void taoABDynamics::_forwardDynamicsOutIn(taoDNode* root, taoABDynamicsData* datah, deVector6* Pah, const deVector6 *Vh)
{
	taoABDynamicsData data;
	deVector6* Pa = root->getABNode()->Pa();
	deVector6* V = root->getABNode()->V();
	deVector6 G;

	root->getABNode()->abInertiaInit(data.Ia);

	root->getABNode()->velocity(*V, data.WxV, *Vh, datah->WxV);

	root->getABNode()->biasForce(*Pa, *V, data.WxV);

	root->getABNode()->gravityForce(G, data.g, datah->g);

	root->getABNode()->externalForce(*Pa, G, *root->force());

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		_forwardDynamicsOutIn(n, &data, Pa, V);

	root->getABNode()->abInertiaDepend(datah->Ia, *Pah, data.Ia, !root->isParentRoot());
}

void taoABDynamics::_inverseDynamicsOutIn(taoDNode* root, taoABDynamicsData2* datah, deVector6* Fh, const deVector6* Vh, const deVector6* Ah)
{
	taoABDynamicsData2 data;
	deVector6* F = root->getABNode()->Pa();
	deVector6* A = root->getABNode()->A();
	deVector6* V = root->getABNode()->V();
	deVector6 P;
	deVector6 G;

	if (!root->getPropagate())
	{
		F->zero();
	}
	else
	{
		root->getABNode()->velocity(*V, data.WxV, *Vh, datah->WxV);

		root->getABNode()->biasForce(P, *V, data.WxV);
		
		root->getABNode()->gravityForce(G, data.g, datah->g);

		root->getABNode()->externalForce(P, G, *root->force());

		root->getABNode()->accelerationOnly(*A, *Ah);

		root->getABNode()->netForce(*F, *A, P);
	}

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		_inverseDynamicsOutIn(n, &data, F, V, A);

	root->getABNode()->force(*Fh, !root->isParentRoot());
}

void taoABDynamics::_accelerationTreeOut(taoDNode* root, const deVector6* Ah)
{
	deVector6* A = root->getABNode()->A();

	root->getABNode()->acceleration(*A, *Ah);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		_accelerationTreeOut(n, A);
}


void taoABDynamics::_velocityDeltaTreeOut(taoDNode* root, const deVector6* dVh, const deVector6* Vh, const deInt dist)
{
	deVector6 *V = root->getABNode()->V();
	deVector6 dV;

	root->getABNode()->velocityDelta(dV, *dVh, dist);
	root->getABNode()->velocityOnly(*V, *Vh);
	
	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		_velocityDeltaTreeOut(n, &dV, V, dist);
}

deFloat taoABDynamics::potentialEnergy(taoDNode* root, const deVector3* gh)
{
	deFloat E;

	E = root->getABNode()->potentialEnergy(*gh, *root->frameGlobal(), *root->mass(), *root->center());

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		E += potentialEnergy(n, gh);

	return E;
}

deFloat taoABDynamics::kineticEnergy(taoDNode* root, const deVector6* Vh)
{
	deFloat E;
	deVector6* V = root->getABNode()->V();

	E = root->getABNode()->kineticEnergy(*V, *Vh);

	for (taoDNode* n = root->getDChild(); n != NULL; n = n->getDSibling())
		E += kineticEnergy(n, V);

	return E;
}
