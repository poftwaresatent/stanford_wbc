//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGInputParameters.h
//!
//! \brief
//! Header file for the input parameters of the Type I On-Line Trajectory
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



#ifndef __TypeIOTGInputParameters__
#define __TypeIOTGInputParameters__

#include <reflexxes_otg/TypeIOTGVector.h>

namespace TypeIOTGMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIOTGInputParameters
//!
//! \brief
//! This class describes the input values for the Type I On-Line Trajectory
//! Generation algorithm as they used internally.
//! 
//! \details
//! It is part of the namespace TypeIOTGMath.
//! 
//! \sa TypeIOTG
//! \sa TypeIOTGOutputParameters
//  ----------------------------------------------------------
class TypeIOTGInputParameters
{

public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGInputParameters(const unsigned int &NumberOfDOFs)
//!
//! \brief Constructor of the class TypeIOTGInputParameters
//!
//! \details
//! It allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param NumberOfDOFs
//! Number of degrees of freedom
//!
//! \sa TypeIOTG
//  ----------------------------------------------------------

	TypeIOTGInputParameters(const unsigned int &NumberOfDOFs)
	{
		this->CurrentPosition			=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->CurrentVelocity			=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->MaxVelocity				=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->MaxAcceleration			=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->TargetPosition			=	new TypeIOTGDoubleVector(NumberOfDOFs);
		this->SelectionVector			=	new TypeIOTGBoolVector  (NumberOfDOFs);

		this->CurrentPosition->Set(0.0)		;
		this->CurrentVelocity->Set(0.0)		;
		this->MaxVelocity->Set(0.0)			;		
		this->MaxAcceleration->Set(0.0)		;
		this->TargetPosition->Set(0.0)		;
		this->SelectionVector->Set(false)	;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGInputParameters(const TypeIOTGInputParameters & IP)
//!
//! \brief
//! Copy constructor of the class TypeIOTGInputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param IP
//! Original object reference
//  ----------------------------------------------------------
	TypeIOTGInputParameters(const TypeIOTGInputParameters & IP)
	{
		this->CurrentPosition			=	new TypeIOTGDoubleVector(IP.CurrentPosition->GetLength()	);
		this->CurrentVelocity			=	new TypeIOTGDoubleVector(IP.CurrentVelocity->GetLength()	);
		this->MaxVelocity				=	new TypeIOTGDoubleVector(IP.MaxVelocity->GetLength()		);
		this->MaxAcceleration			=	new TypeIOTGDoubleVector(IP.MaxAcceleration->GetLength()	);
		this->TargetPosition			=	new TypeIOTGDoubleVector(IP.TargetPosition->GetLength()		);
		this->SelectionVector			=	new TypeIOTGBoolVector  (IP.SelectionVector->GetLength()	);

		this->CurrentPosition	=	IP.CurrentPosition	;
		this->CurrentVelocity	=	IP.CurrentVelocity	;
		this->MaxVelocity		=	IP.MaxVelocity		;	
		this->MaxAcceleration	=	IP.MaxAcceleration	;
		this->TargetPosition	=	IP.TargetPosition	;
		this->SelectionVector	=	IP.SelectionVector	;

	}

//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGInputParameters &operator = (const TypeIOTGInputParameters &IP)
//!
//! \brief
//! Copy operator
//!
//! \param IP
//! Set of input parameters
//  ----------------------------------------------------------
	TypeIOTGInputParameters &operator = (const TypeIOTGInputParameters &IP)
	{
		this->CurrentPosition	=	IP.CurrentPosition	;
		this->CurrentVelocity	=	IP.CurrentVelocity	;
		this->MaxVelocity		=	IP.MaxVelocity		;	
		this->MaxAcceleration	=	IP.MaxAcceleration	;
		this->TargetPosition	=	IP.TargetPosition	;
		this->SelectionVector	=	IP.SelectionVector	;
		return *this;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIOTGInputParameters(void)
//!
//! \brief
//! Destructor of class TypeIOTGInputParameters, frees heap memory
//  ----------------------------------------------------------
	~TypeIOTGInputParameters(void)
	{
		delete this->CurrentPosition	;
		delete this->CurrentVelocity	;
		delete this->MaxVelocity		;	
		delete this->MaxAcceleration	;
		delete this->TargetPosition		;
		delete this->SelectionVector	;

		this->CurrentPosition		=	NULL;
		this->CurrentVelocity		=	NULL;
		this->MaxVelocity			=	NULL;
		this->MaxAcceleration		=	NULL;
		this->TargetPosition		=	NULL;
		this->SelectionVector		=	NULL;
	}


	TypeIOTGDoubleVector*	CurrentPosition	;
	TypeIOTGDoubleVector*	CurrentVelocity	;
	TypeIOTGDoubleVector*	MaxAcceleration	;
	TypeIOTGDoubleVector*	MaxVelocity		;
	TypeIOTGDoubleVector*	TargetPosition	;
	TypeIOTGBoolVector*		SelectionVector	;
};

}	// namespace

#endif
