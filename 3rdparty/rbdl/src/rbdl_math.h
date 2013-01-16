/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#include <rbdl_config.h>

#ifdef RBDL_USE_SIMPLE_MATH
  #include "SimpleMath/SimpleMath.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3_t;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector_t;
	typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix_t;

	typedef SimpleMath::Dynamic::Matrix<double> MatrixN_t;
	typedef SimpleMath::Dynamic::Matrix<double> VectorN_t;

#else
	#define EIGEN_DEFAULT_TO_ROW_MAJOR
	#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

	#include "Eigen/Dense"
	#include "Eigen/StdVector"

	typedef Eigen::Matrix< double, 3, 1> Vector3_t;
	typedef Eigen::Matrix< double, 3, 3> Matrix3_t;

	typedef Eigen::VectorXd VectorN_t;
	typedef Eigen::MatrixXd MatrixN_t;

	typedef Eigen::Matrix< double, 6, 1> SpatialVector_t;
	typedef Eigen::Matrix< double, 6, 6> SpatialMatrix_t;
#endif

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
	typedef Vector3_t Vector3d;
	typedef Matrix3_t Matrix3d;
	typedef SpatialVector_t SpatialVector;
	typedef SpatialMatrix_t SpatialMatrix;
	typedef VectorN_t VectorNd;
	typedef MatrixN_t MatrixNd;

	inline Matrix3d mkMatrix3d(double a, double b, double c, double d, double e, double f, double g, double h, double i) {
	  Matrix3d mx;
	  mx << a, b, c, d, e, f, g, h, i;
	  return mx;
	}
	  
	inline SpatialVector mkSpatialVector(double a, double b, double c, double d, double e, double f) {
	  SpatialVector sv;
	  sv << a, b, c, d, e, f;
	  return sv;
	}
	
	inline SpatialMatrix mkSpatialMatrix(double a1, double b1, double c1, double d1, double e1, double f1,
					     double a2, double b2, double c2, double d2, double e2, double f2,
					     double a3, double b3, double c3, double d3, double e3, double f3,
					     double a4, double b4, double c4, double d4, double e4, double f4,
					     double a5, double b5, double c5, double d5, double e5, double f5,
					     double a6, double b6, double c6, double d6, double e6, double f6) {
	  SpatialMatrix sm;
	  sm <<
	    a1, b1, c1, d1, e1, f1,
	    a2, b2, c2, d2, e2, f2,
	    a3, b3, c3, d3, e3, f3,
	    a4, b4, c4, d4, e4, f4,
	    a5, b5, c5, d5, e5, f5,
	    a6, b6, c6, d6, e6, f6;
	  return sm;
	}
	
	inline void set(Vector3d & v3, double a, double b, double c) {
	  v3[0] = a;
	  v3[1] = b;
	  v3[2] = c;
	}
	
	inline void set(Matrix3d & m3, double a, double b, double c, double d, double e, double f, double g, double h, double i) {
	  m3 << a, b, c, d, e, f, g, h, i;
	}
	
	inline void set(SpatialVector & sv, double a, double b, double c, double d, double e, double f) {
	  sv << a, b, c, d, e, f;
	}
	
	inline void set(SpatialMatrix & sm,
			double a1, double b1, double c1, double d1, double e1, double f1,
			double a2, double b2, double c2, double d2, double e2, double f2,
			double a3, double b3, double c3, double d3, double e3, double f3,
			double a4, double b4, double c4, double d4, double e4, double f4,
			double a5, double b5, double c5, double d5, double e5, double f5,
			double a6, double b6, double c6, double d6, double e6, double f6) {
	  sm <<
	    a1, b1, c1, d1, e1, f1,
	    a2, b2, c2, d2, e2, f2,
	    a3, b3, c3, d3, e3, f3,
	    a4, b4, c4, d4, e4, f4,
	    a5, b5, c5, d5, e5, f5,
	    a6, b6, c6, d6, e6, f6;
	}

} /* Math */

} /* RigidBodyDynamics */

#include "SpatialAlgebraOperators.h"

// If we use Eigen3 we have to create specializations of the STL
// std::vector such that the alignment is done properly.
#ifndef RBDL_USE_SIMPLE_MATH
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialVector)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialMatrix)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialTransform)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialRigidBodyInertia)
#endif

#endif /* _MATHWRAPPER_H */
