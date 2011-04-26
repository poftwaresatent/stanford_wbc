// -*- tab-width: 2 -*-
//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTG.h
//!
//! \brief
//! Header file for the class of the Type I On-Line Trajectory Generator
//!
//! \details
//! This file contains the user interface of the Type I On-Line
//! Trajectory Generation algorithm. Besides the constructor of the class
//! TypeIOTG, the two most important methods are 
//! TypeIOTG::GetNextMotionState_Position and
//! TypeIOTG::GetNextMotionState_Velocity.
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


#ifndef __TypeIOTG__
#define __TypeIOTG__

#include <reflexxes_otg/TypeIOTGVector.h>
#include <reflexxes_otg/TypeIOTGInputParameters.h>
#include <reflexxes_otg/TypeIOTGOutputParameters.h>
#include <reflexxes_otg/TypeIOTGPolynomial.h>




//  ---------------------- Doxygen info ----------------------
//! \class TypeIOTG
//! 
//! \brief
//! This class realizes the interface of the Reflexxes Type I
//! On-Line Trajectory Generation Library
//!
//! \details
//! The algorithm works with an arbitrary number of degrees of freedom.
//! It can handle and proceed with any state of
//! motion, that is, a new motion trajectory can be generated in any
//! situation after any sensor event within the same control cycle.
//! Further and detailed information can be found in\n
//! \n
//! T. Kroeger.\n
//! On-Line Trajectory Generation in Robotic Systems.\n
//! Springer Tracts in Advanced Robotics, Vol. 58, Springer, January 2010.
//  ----------------------------------------------------------
class TypeIOTG
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTG(const unsigned int &NoOfDOFs, const double &CycleTimeInSeconds)
//! 
//! \brief
//! Constructor of the class TypeIOTG
//! 
//! \param NoOfDOFs
//! Indicates the number of degrees of freedom
//! 
//! \param CycleTimeInSeconds
//! Indicates the cycle time in seconds for the generator. A typical
//! value is 0.001 (seconds).
//  ----------------------------------------------------------
	TypeIOTG				(		const unsigned int			&NoOfDOFs
								,	const double				&CycleTimeInSeconds  );
	
//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIOTG(void)
//! 
//! \brief
//! Destructor of the class TypeIOTG
//  ----------------------------------------------------------
	~TypeIOTG				(		void	);


//  ---------------------- Doxygen info ----------------------
//! \enum TypeIOTGResult
//! 
//! \brief
//! Result values of the methods TypeIOTG::GetNextMotionState_Position()
//! and TypeIOTG::GetNextMotionState_Velocity()
//  ----------------------------------------------------------
	enum TypeIOTGResult
	{
		//! \brief
		//! A general error.
		OTG_ERROR								=	-1,
		//! \brief
		//! The specified maximum velocity value of at least one
		//! degree of freedom is below the threshold of OTG_MIN_VALUE_FOR_MAXVELOCITY.
		OTG_MAX_VELOCITY_ERROR					=	-200,
		//! \brief
		//! The specified maximum acceleration value of at least one
		//! degree of freedom is below the threshold of OTG_MIN_VALUE_FOR_MAXACCELERATION.
		OTG_MAX_ACCELERATION_ERROR				=	-201,
		//! \brief
		//! The on-line trajectory generation algorithm is working; the final
		//! state of motion has not been reached yet.
		OTG_WORKING								=	0,
		//! \brief
		//! The desired final state of motion has been reached.
		OTG_FINAL_STATE_REACHED					=	1,
	};


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Position(const double* CurrentPosition, const double* CurrentVelocity, const double* MaxVelocity, const double* MaxAcceleration, const double* TargetPosition, const bool* SelectionVector, double* NewPosition, double* NewVelocity)
//! 
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ \vec{V}_{i+1} \f$ values
//! 
//! \param CurrentPosition
//! The current position \f$ \vec{P}_{i} \f$ (position of the last
//! cycle, given by the user)
//! 
//! \param CurrentVelocity
//! The current velocity \f$ \vec{V}_{i} \f$ (velocity of the last
//! cycle, given by the user)
//!
//! \param MaxVelocity
//! The maximum velocity \f$ \vec{V}^{\,max}_{i} \f$ (given by the user)
//!
//! \param MaxAcceleration
//! The maximum acceleration \f$ \vec{A}^{\,max}_{i} \f$ (given by the user)
//!
//! \param TargetPosition
//! The target Position \f$ \vec{P}^{\,trgt}_{i} \f$ (given by the user)
//!
//! \param
//! SelectionVector The selection vector \f$ \vec{S}_{i} \f$, which
//! determines, which DOFs are position controlled by the OTG (of
//! the current control cycle, given by the user)
//!
//! \param NewPosition
//! Pointer to an array of double values for the new position
//! \f$ \vec{P}_{i+1} \f$ (position of the current control cycle,
//! to be calculated)
//!
//! \param NewVelocity
//! Pointer to an array of double values for the new velocity
//! \f$ \vec{V}_{i+1} \f$ (velocity of the current control cycle,
//! to be calculated)
//!
//! \return OnlineTrajectoryGenerator::OTG_MAX_VELOCITY_ERROR
//! \return OnlineTrajectoryGenerator::OTG_MAX_ACCELERATION_ERROR
//! \return OnlineTrajectoryGenerator::OTG_ERROR
//! \return OnlineTrajectoryGenerator::OTG_WORKING 
//! \return OnlineTrajectoryGenerator::OTG_FINAL_STATE_REACHED
//!
//! \sa TypeIOTG::TypeIOTGResult
//  ----------------------------------------------------------
	int GetNextMotionState_Position	(		const double*	CurrentPosition
										,	const double*	CurrentVelocity
										,	const double*	MaxVelocity
										,	const double*	MaxAcceleration
										,	const double*	TargetPosition
										,	const bool*		SelectionVector
										,	double*			NewPosition
										,	double*			NewVelocity	);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetNextMotionState_Velocity(const double* CurrentPosition, const double* CurrentVelocity, const double* MaxAcceleration, const double* TargetVelocity, const bool* SelectionVector, double* NewPosition, double* NewVelocity)
//!
//! \brief
//! Calculates new position \f$ \vec{P}_{i+1} \f$ and velocity
//! \f$ (\vec{V}_{i+1} \f$ values for position-based trajectory generation
//!
//! \param CurrentPosition
//! The current position \f$ (\vec{P}_{i} \f$ (position of the last
//! cycle, given by the user)
//!
//! \param CurrentVelocity
//! The current velocity \f$ (\vec{V}_{i} \f$ (velocity of the last
//! cycle, given by the user)
//!
//! \param MaxAcceleration
//! The maximum acceleration \f$ \vec{A}^{\,max}_{i} \f$ (given by the user)
//!
//! \param TargetVelocity
//! The target velocity \f$ (\vec{V}_{i}^{\,trgt} \f$ (velocity to be
//! reached, given by the user)
//!
//! \param SelectionVector
//! The selection vector \f$ \vec{s}_{i} \f$, which determines, which DOFs
//! are position controlled by the OTG (of the current control cycle,
//! given by the user)
//!
//! \param NewPosition
//! Pointer to an array of double values for the new position
//! \f$ (\vec{p}_{i} \f$ (position of the current control cycle,
//! to be calculated)
//!
//! \param NewVelocity
//! Pointer to an array of double values for the new velocity
//! \f$ (\vec{v}_{i} \f$ (velocity of the current control cycle,
//! to be calculated)
//!
//! \return OnlineTrajectoryGenerator::OTG_MAX_ACCELERATION_ERROR
//! \return OnlineTrajectoryGenerator::OTG_ERROR
//! \return OnlineTrajectoryGenerator::OTG_WORKING 
//! \return OnlineTrajectoryGenerator::OTG_FINAL_STATE_REACHED
//!
//! \sa TypeIOTG::TypeIOTGResult
//  ----------------------------------------------------------
	int GetNextMotionState_Velocity	(		const double*	CurrentPosition
										,	const double*	CurrentVelocity
										,	const double*	MaxAcceleration
										,	const double*	TargetVelocity
										,	const bool*		SelectionVector
										,	double*			NewPosition
										,	double*			NewVelocity	);


//  ---------------------- Doxygen info ----------------------
//! \fn double GetExecutionTime(void) const
//!
//! \brief
//! Returns the time in seconds that is required to time-optimally
//! reach the desired target state of motion
//!
//! \details
//! If this method is called after the execution of the method
//! TypeIOTG::GetNextMotionState_Position(), the time value for the
//! position-based trajectory will be returned. If this method is
//! called after the execution of the method
//! TypeIOTG::GetNextMotionState_Velocity(), the time value for the
//! velocity-based trajectory will be returned, i.e., the time until
//! which the last degree of freedom will reach its target velocity.
//! For the case, the final and desired state of motion has already
//! been reached (i.e., the result of
//! TypeIOTG::GetNextMotionState_Position()
//! or TypeIOTG::GetNextMotionState_Velocity() was
//! TypeIOTG::OTG_FINAL_STATE_REACHED), this method returns a zero
//! value.
//!
//! \return The value of the execution time in seconds. If neither the
//! method TypeIOTG::GetNextMotionState_Position()
//! nor the method TypeIOTG::GetNextMotionState_Velocity() was not
//! called before, the returned value is -1.0.
//!
//! \sa TypeIOTG::GetNextMotionState_Position()
//! \sa TypeIOTG::GetNextMotionState_Velocity()
//  ----------------------------------------------------------
	double	GetExecutionTime(void) const;


private:

	double CalculateMinimumSynchronizationTime(const TypeIOTGMath::TypeIOTGInputParameters &IP) const;

	void SynchronizeTrajectory(		const TypeIOTGMath::TypeIOTGInputParameters		&IP
								,	const double									&SynchronizationTime
								,	TypeIOTGMath::TypeIMotionPolynomials			*PolynomialArray);

	int CalculateOutputValues(		const TypeIOTGMath::TypeIMotionPolynomials		*PolynomialArray
								,	const TypeIOTGMath::TypeIOTGBoolVector			&SelectionVector
								,	TypeIOTGMath::TypeIOTGOutputParameters			*OP);

	int								ReturnValue
								,	NumberOfDOFs;

	double							CycleTime										// in seconds
								,	InternalClockInSeconds
								,	TrajectoryExecutionTimeForTheUser;				// in seconds

	TypeIOTGMath::TypeIOTGInputParameters		*CurrentInputParameters;
		
	TypeIOTGMath::TypeIOTGOutputParameters		*OutputParameters;

	TypeIOTGMath::TypeIMotionPolynomials		*Polynomials;

};



#endif
