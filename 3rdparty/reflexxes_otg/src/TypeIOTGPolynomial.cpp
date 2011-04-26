//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGPolynomial.cpp
//!
//! \brief
//! Implementation file for polynomials designed for the Type I On-Line Trajectory
//! Generation algorithm
//!
//! \sa TypeIOTGPolynomial.h
//! \n
//! \n
//! \b License 
//! \n
//! \n
//! This file is part of the Reflexxes Type I OTG Library.
//! \n\n
//! The Reflexxes Type I OTG Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Reflexxes Type I OTG Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Reflexxes Type I OTG Library. If not, see 
//! <http://www.gnu.org/licenses/>.
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date June 2010
//! 
//! \version 1.0
//!
//!	\author Torsten Kroeger, <info@reflexxes.com> \n
//!	
//!
//! \note Copyright (C) 2010 Reflexxes GmbH.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



#include <reflexxes_otg/TypeIOTGPolynomial.h>
#include <reflexxes_otg/TypeIOTGMath.h>


//************************************************************************************
// Constructor


TypeIOTGMath::TypeIOTGPolynomial::TypeIOTGPolynomial()
{
	a0		= 0.0;
	a1		= 0.0;
	a2		= 0.0;
	DeltaT	= 0.0;
	Degree	= 0;
}


//************************************************************************************
// Destructor

TypeIOTGMath::TypeIOTGPolynomial::~TypeIOTGPolynomial()
{}


//************************************************************************************
// SetCoefficients()
// f(x) = a_2 * (t - DeltaT)^2 + a_1 * (t - DeltaT) + a_0

void TypeIOTGMath::TypeIOTGPolynomial::SetCoefficients(		const double	&Coeff2
														,	const double	&Coeff1
														,	const double	&Coeff0
														,	const double	&Diff)
{
	a0		= Coeff0;
	a1		= Coeff1;
	a2		= Coeff2;
	DeltaT	= Diff;

	if (a2 != 0.0)
	{
		Degree = 2;
		return;
	}

	if (a1 != 0.0)
	{
		Degree = 1;
		return;
	}

	Degree = 0;
	return;
}


//*******************************************************************************************
// CalculateValue()
// calculates f(t)

double TypeIOTGMath::TypeIOTGPolynomial::CalculateValue(const double &t) const
{
	return(	((Degree == 2)?
			(a2 * (t - DeltaT) * (t - DeltaT) + a1 * (t - DeltaT) + a0):
			((Degree == 1)?
			(a1 * (t - DeltaT) + a0):
			(a0))));
}
