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

// modification history
// --------------------
//
// 07/11/06: Luis Sentis. lsentis@ai.stanford.edu: 
//		Modified _Read and _Write functions to work on branching robots.


#include <tao/dynamics/taoDynamics.h>
#include "taoDNode.h"
#include "taoABDynamics.h"
#include <tao/utility/TaoDeMassProp.h>
#include <tao/dynamics/taoJoint.h>
#include <assert.h>
#include <string.h>
#include <vector>

void taoDynamics::initialize(taoDNode* root)
{
	taoABDynamics::updateLocalXTreeOut(root);
	taoABDynamics::resetInertiaTreeOut(root);
	taoABDynamics::resetFlagTreeOut(root);
	updateTransformation(root);
}

void taoDynamics::reset(taoDNode* r)
{
	r->sync(r->frameHome());
	for (taoDNode* n = r->getDChild(); n != NULL; n = n->getDSibling())
		reset(n);
}

void taoDynamics::updateTransformation(taoDNode* r)
{
	r->updateFrame();
	for (taoDNode* n = r->getDChild(); n != NULL; n = n->getDSibling())
		updateTransformation(n);
}

void taoDynamics::integrate(taoDNode* r, deFloat dt)
{
	r->integrate(dt);
	for (taoDNode* n = r->getDChild(); n != NULL; n = n->getDSibling())
		integrate(n, dt);
}

void taoDynamics::globalJacobian(taoDNode* root)
{
	taoABDynamics::globalJacobianOut(root);
}

void taoDynamics::invDynamics(taoDNode* root, const deVector3* gravity)
{
	taoABDynamics::updateLocalXTreeOut(root); // YYY
	deVector3 g;
	g.inversedMultiply(root->frameGlobal()->rotation(), *gravity);
	deVector6 A = *root->acceleration();
	root->acceleration()->zero();
	taoABDynamics::inverseDynamics(root, &g);
	*root->acceleration() = A;
}

void taoDynamics::fwdDynamics(taoDNode* root, const deVector3* gravity)
{
	taoABDynamics::updateLocalXTreeOut(root); // YYY
	deVector3 g;
	g.inversedMultiply(root->frameGlobal()->rotation(), *gravity);
	taoABDynamics::forwardDynamics(root, &g);
}

void taoDynamics::impulse(taoDNode* contact, const deVector3* contactPodeInt, const deVector3* impulseVector)
{
	taoABDynamics::forwardDynamicsImpulse(contact, contactPodeInt, impulseVector, 0);
}

void taoDynamics::impulseDist(taoDNode* contact, const deVector3* contactPodeInt, const deVector3* impulseVector)
{
	taoABDynamics::forwardDynamicsImpulse(contact, contactPodeInt, impulseVector, 1);
}

// Fc = X F
// X = [ R 0; rx R R ] = [ 1 0; rx 1 ] 
// Fc = X F = [ 1 0; rx 1 ] [ F; 0] = [ F; rx F]
void taoDynamics::force(taoDNode* contact, const deVector3* contactPodeInt, const deVector3* forceVector)
{
	(*contact->force())[0] = *forceVector;
	(*contact->force())[1].crossMultiply(*contactPodeInt, *forceVector);
}

void taoDynamics::resetMass(taoDNode* node, const deFloat mass)
{
	deMassProp prop;
	prop.set(node->mass(), node->center(), node->inertia());
	prop.scale(mass);
	prop.get(node->mass(), node->center(), node->inertia());
	taoABDynamics::resetInertia(node);
}

deFloat taoDynamics::potentialEnergy(taoDNode* rootNode, const deVector3* gravity)
{
	return taoABDynamics::potentialEnergy(rootNode, gravity);
}

deFloat taoDynamics::kineticEnergy(taoDNode* rootNode)
{
	return taoABDynamics::kineticEnergy(rootNode);
}

// find dof : degrees of freedom
deInt taoDynamics::computeDOF(taoDNode* root)
{
	taoJoint* j;
	taoDNode* n;
	deInt dof = 0;

	for (j = root->getJointList(); j != NULL; j = j->getNext())
		dof += j->getDOF();

	for (n = root->getDChild(); n != NULL; n = n->getDSibling())
		dof += computeDOF(n);

	return dof;
}

// Ainv = inverse of mass matrix
// in order to get A, B, G
void taoDynamics::computeG(taoDNode* root, deVector3* gravity, const deInt dof, deFloat* G)
{
	assert(dof == computeDOF(root));

	// XXXX: ddq, dq, and tau "only" serve to store the old state,
	// which will get restored after the computation... shouldn't
	// there be a better way to do this, e.g. by externalizing the
	// state?
	//
	// Also note that we use vector<> to implement a dynamically
	// sized array, in order to be more portable. GCC allows
	// "deFloat ddq[dof]" but that is actually a GNU extensions
	// and does not build i.e. under Windows. In order to use the
	// vector<> as a pointer, we retrieve the address of its first
	// element like "&ddq[0]", which is guaranteed to be portable.
	std::vector<deFloat> ddq(dof);
	std::vector<deFloat> dq(dof);
	std::vector<deFloat> tau(dof);
	
	std::vector<deFloat> zero(dof, 0);

	// tau = A * ddq + b + g
	// ddq = 0;
	// dq = 0
	// tau = g
	// zeroDDQ();
	// zeroDQ();
	// invDynamics();
	// G = tau;

	_Read(root, &ddq[0], TAO_DDQ);
	_Read(root, &dq[0], TAO_DQ);
	_Read(root, &tau[0], TAO_TAU);
	
	_Write(root, &zero[0], TAO_DDQ);
	_Write(root, &zero[0], TAO_DQ);

	invDynamics(root, gravity);
	_Read(root, G, TAO_TAU);

	_Write(root, &ddq[0], TAO_DDQ);
	_Write(root, &dq[0], TAO_DQ);
	_Write(root, &tau[0], TAO_TAU);
}

void taoDynamics::computeB(taoDNode* root, const deInt dof, deFloat* B)
{
	assert(dof == computeDOF(root));

	// see also comments in computeG()
	std::vector<deFloat> ddq(dof);
	std::vector<deFloat> tau(dof);

	std::vector<deFloat> zero(dof, 0);

	// tau = A * ddq + b + g
	// ddq = 0;
	// g = 0
	// tau = b
	// zeroDDQ();
	// g = 0;
	// invDynamics();
	// B = tau;

	deVector3 g;

	g.zero();

	_Read(root, &ddq[0], TAO_DDQ);
	_Read(root, &tau[0], TAO_TAU);
	_Write(root, &zero[0], TAO_DDQ);

	invDynamics(root, &g);
	_Read(root, B, TAO_TAU);

	_Write(root, &ddq[0], TAO_DDQ);
	_Write(root, &tau[0], TAO_TAU);
}

// Li = J Ai Jt
void taoDynamics::computeOpSpaceInertiaMatrixInv(taoDNode* root, const deFloat* J, const deInt row, const deInt dof, const deFloat* Ainv, deFloat* Linv)
{
	std::vector<deFloat> L(row * dof);

	int i, j, k;

	assert(dof == computeDOF(root));

	// L = J * Ainv
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < dof; j++)
		{
			L[i * dof + j] = 0;

			for (k = 0; k < dof; k++)
				L[i * dof + j] += J[i * dof + k] * Ainv[k * dof + j];
		}
	}
	// Linv = L * Jt
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			Linv[i * dof + j] = 0;

			for (k = 0; k < dof; k++)
				Linv[i * dof + j] += L[i * dof + k] * J[j * dof + k];
		}
	}
}

void taoDynamics::computeA(taoDNode* root, const deInt dof, deFloat* A)
{
	assert(dof == computeDOF(root));

	// see also comments in computeG()
	std::vector<deFloat> ddq2(dof, 0);
	
	std::vector<deFloat> ddq(dof);
	std::vector<deFloat> dq(dof);
	std::vector<deFloat> tau(dof);

	std::vector<deFloat> zero(dof, 0);

	deVector3 g;
	deInt i;

	// tau = A * ddq + b + g
	// dq = 0;
	// g = 0
	// tau = A * ddq
	// zeroDQ();
	// g = 0;
	// invDynamics();
	// Ai = taui
	// tau is i_th col of Ai

	g.zero();

	_Read(root, &ddq[0], TAO_DDQ);
	_Read(root, &dq[0], TAO_DQ);
	_Read(root, &tau[0], TAO_TAU);

	_Write(root, &zero[0], TAO_DQ);

	for (i = 0; i < dof; i++)
	{
		ddq2[i] = 1;
		_Write(root, &ddq2[0], TAO_DDQ);
		invDynamics(root, &g);
		_Read(root, A + dof * i, TAO_TAU);
		ddq2[i] = 0;
	}

	_Write(root, &ddq[0], TAO_DDQ);
	_Write(root, &dq[0], TAO_DQ);
	_Write(root, &tau[0], TAO_TAU);
}

void taoDynamics::computeAinv(taoDNode* root, const deInt dof, deFloat* Ainv)
{
	assert(dof == computeDOF(root));

	// see also comments in computeG()
	std::vector<deFloat> tau2(dof, 0);
	
	std::vector<deFloat> tau(dof);
	std::vector<deFloat> ddq(dof);
	std::vector<deFloat> dq(dof);
	
	std::vector<deFloat> zero(dof, 0);

	deVector3 g;
	deInt i;

	// tau = A * ddq + b + g
	// ddq = Ainv(tau - b - g)
	// dq = 0;
	// g = 0
	// ddq = Ainv * tau
	// zeroDQ();
	// g = 0;
	// fwdDynamics();
	// Ainvi = ddqi
	// ddq is i_th col of Ainv
	g.zero();
	_Read(root, &tau[0], TAO_TAU);
	_Read(root, &ddq[0], TAO_DDQ);
	_Read(root, &dq[0], TAO_DQ);
	_Write(root, &zero[0], TAO_DQ);

	for (i = 0; i < dof; i++)
	{
		tau2[i] = 1;
		_Write(root, &tau2[0], TAO_TAU);
		fwdDynamics(root, &g);
		_Read(root, Ainv + dof * i, TAO_DDQ);
		tau2[i] = 0;
	}

	_Write(root, &tau[0], TAO_TAU);
	_Write(root, &ddq[0], TAO_DDQ);
	_Write(root, &dq[0], TAO_DQ);
}

void taoDynamics::_Read(taoDNode* root, deFloat* v, flagType type)
{
	taoJoint* j;
	taoDNode* n;

	deInt id = root->getID();

	for (j = root->getJointList(); j != NULL; j = j->getNext())
	{
		switch (type)
		{
		case TAO_DDQ:
			//j->getDDQ(v); // was
			j->getDDQ(&v[id]);
			break;
		case TAO_DQ:
			//j->getDQ(v); // was
			j->getDQ(&v[id]);
			break;
		case TAO_TAU:
			//j->getTau(v); //was
			j->getTau(&v[id]);
			break;
		}
		//v += j->getDOF();
	}

	for (n = root->getDChild(); n != NULL; n = n->getDSibling())
		_Read(n, v, type);
}

void taoDynamics::_Write(taoDNode* root, deFloat* v, flagType type)
{
	taoJoint* j;
	taoDNode* n;

	deInt id = root->getID();

	for (j = root->getJointList(); j != NULL; j = j->getNext())
	{
		switch (type)
		{
		case TAO_DDQ:
			//j->setDDQ(v); //was
			j->setDDQ(&v[id]);
			break;
		case TAO_DQ:
			//j->setDQ(v); //was
			j->setDQ(&v[id]);
			break;
		case TAO_TAU:
			//j->setTau(v); //was
			j->setTau(&v[id]);
			break;
		}
		//v += j->getDOF();
	}

	for (n = root->getDChild(); n != NULL; n = n->getDSibling())
		_Write(n, v, type);
}

