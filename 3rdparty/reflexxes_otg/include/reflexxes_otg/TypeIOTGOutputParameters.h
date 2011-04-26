//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGOutputParameters.h
//!
//! \brief
//! Header file for the output parameters of the Type I On-Line Trajectory
//! Generation algorithm
//!
//! \details
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


#ifndef __TypeIOTGOutputParameters__
#define __TypeIOTGOutputParameters__


#include <reflexxes_otg/TypeIOTGVector.h>


namespace TypeIOTGMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIOTGOutputParameters
//!
//! \brief
//! This class describes the Output values for the Type I On-Line 
//! Trajectory Generation algorithm as they used internally.
//! 
//! \details
//! It is part of the namespace TypeIOTGMath.
//! 
//! \sa TypeIOTG
//! \sa TypeIOTGInputParameters
//  ----------------------------------------------------------
class TypeIOTGOutputParameters
{

public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGOutputParameters(const unsigned int &NumberOfDOFs)
//!
//! \brief
//! Constructor of the class TypeIOTGOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param NumberOfDOFs
//! Number of degrees of freedom
//  ----------------------------------------------------------

	TypeIOTGOutputParameters(const unsigned int &NumberOfDOFs)
	{
		this->NewPosition			=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->NewVelocity			=	new TypeIOTGDoubleVector(NumberOfDOFs);

		this->NewPosition->Set(0.0);
		this->NewVelocity->Set(0.0);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGOutputParameters(const TypeIOTGOutputParameters & OP)
//!
//! \brief
//! Copy constructor of the class TypeIOTGOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! \brief A call of this method is \em not real-time capable!!!
//!
//! \param OP
//! Original object reference
//  ----------------------------------------------------------
	TypeIOTGOutputParameters(const TypeIOTGOutputParameters & OP)
	{
		this->NewPosition	=	new TypeIOTGDoubleVector(OP.NewPosition->GetLength());
		this->NewVelocity	=	new TypeIOTGDoubleVector(OP.NewVelocity->GetLength());

		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGOutputParameters &operator = (const TypeIOTGOutputParameters &OP)
//!
//! \brief
//! Copy operator
//!
//! \param OP
//! Set of output parameters
//  ----------------------------------------------------------
	TypeIOTGOutputParameters &operator = (const TypeIOTGOutputParameters &OP)
	{
		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;
		return *this;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIOTGOutputParameters(void)
//!
//! \brief
//! Destructor of class TypeIOTGOutputParameters, frees heap memory
//  ----------------------------------------------------------
	~TypeIOTGOutputParameters(void)
	{
		delete this->NewPosition	;
		delete this->NewVelocity	;

		this->NewPosition	=	NULL;
		this->NewVelocity	=	NULL;
	}

	TypeIOTGDoubleVector*	NewPosition;
	TypeIOTGDoubleVector*	NewVelocity;
};



}	// namespace

#endif
