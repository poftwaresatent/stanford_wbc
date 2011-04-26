//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTG.cpp
//!
//! \brief
//! Implementation file for the class of the Type I On-Line Trajectory Generator
//!
//! \sa TypeIOTG.h
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

#include <reflexxes_otg/TypeIOTG.h>
#include <reflexxes_otg/TypeIOTGPolynomial.h>
#include <reflexxes_otg/TypeIOTGMath.h>
#include <reflexxes_otg/TypeIOTGVector.h>
#include <reflexxes_otg/TypeIOTGInputParameters.h>
#include <reflexxes_otg/TypeIOTGOutputParameters.h>
#include <reflexxes_otg/TypeIOTGDecision.h>
#include <reflexxes_otg/TypeIOTGProfiles.h>
#include <string.h>
#include <math.h>

using namespace TypeIOTGMath;

//************************************************************************************
// Constructor


TypeIOTG::TypeIOTG(const unsigned int &NoOfDOFs, const double &CycleTimeInSeconds)
{
	this->CycleTime									=	CycleTimeInSeconds;
	this->NumberOfDOFs								=	NoOfDOFs;
	this->ReturnValue								=	TypeIOTG::OTG_FINAL_STATE_REACHED;
	this->TrajectoryExecutionTimeForTheUser			=	-1.0;
	this->InternalClockInSeconds					=	0.0;

	this->CurrentInputParameters					=	new TypeIOTGInputParameters	(NumberOfDOFs);
	this->OutputParameters							=	new TypeIOTGOutputParameters(NumberOfDOFs);

	this->Polynomials								=	new TypeIMotionPolynomials	[NumberOfDOFs];
}


//************************************************************************************
// Destructor

TypeIOTG::~TypeIOTG()
{
	delete(this->CurrentInputParameters		);
	delete(this->OutputParameters			);
	delete[](this->Polynomials				);
}


//************************************************************************************
// GetNextMotionState_Position

int TypeIOTG::GetNextMotionState_Position(		const double*	CurrentPosition
											,	const double*	CurrentVelocity
											,	const double*	MaxVelocity
											,	const double*	MaxAcceleration
											,	const double*	TargetPosition
											,	const bool*		SelectionVector
											,	double*			NewPosition
											,	double*			NewVelocity	)
{
	int				i							=	0
				,	ReturnValue					=	TypeIOTG::OTG_ERROR;

	double			MinimumSynchronizationTime	=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		(*this->CurrentInputParameters->SelectionVector)	[i]	=	SelectionVector	[i];
		if (SelectionVector[i])
		{
			if (MaxVelocity[i] < OTG_MIN_VALUE_FOR_MAXVELOCITY)
			{
				return (TypeIOTG::OTG_MAX_VELOCITY_ERROR);
			}
			if (MaxAcceleration[i] < OTG_MIN_VALUE_FOR_MAXACCELERATION)
			{
				return (TypeIOTG::OTG_MAX_ACCELERATION_ERROR);
			}

			(*this->CurrentInputParameters->CurrentPosition	)	[i]	=	CurrentPosition	[i];
			(*this->CurrentInputParameters->CurrentVelocity	)	[i]	=	CurrentVelocity	[i];
			(*this->CurrentInputParameters->MaxVelocity		)	[i]	=	MaxVelocity		[i];
			(*this->CurrentInputParameters->MaxAcceleration	)	[i]	=	MaxAcceleration	[i];
			(*this->CurrentInputParameters->TargetPosition	)	[i]	=	TargetPosition	[i];
		}
	}

	// *******************************************************************************
	// * Step 1: Calculate the minimum possile synchronization time
	// *******************************************************************************

	MinimumSynchronizationTime = TypeIOTG::CalculateMinimumSynchronizationTime(*(this->CurrentInputParameters));

	// *******************************************************************************
	// * Step 2: Synchronize all selected degrees of freedom
	// *******************************************************************************

	TypeIOTG::SynchronizeTrajectory(		*(this->CurrentInputParameters)
										,	MinimumSynchronizationTime
										,	this->Polynomials);

	// *******************************************************************************
	// * Step 3: Calculate output values
	// *******************************************************************************

	ReturnValue = TypeIOTG::CalculateOutputValues(		this->Polynomials
													,	*(this->CurrentInputParameters->SelectionVector)
													,	OutputParameters);

	TrajectoryExecutionTimeForTheUser = MinimumSynchronizationTime;

	if (ReturnValue != TypeIOTG::OTG_ERROR)
	{
		for (i = 0; i < this->NumberOfDOFs; i++)
		{
			if (SelectionVector[i])
			{
				NewPosition[i] = (*OutputParameters->NewPosition)[i];
				NewVelocity[i] = (*OutputParameters->NewVelocity)[i];
			}
		}
	}

	return (ReturnValue);
}


//************************************************************************************
// CalculateMinimumSynchronizationTime()

double TypeIOTG::CalculateMinimumSynchronizationTime(const TypeIOTGInputParameters &IP) const
{
	int				i							=	0;

	double			CurrentPosition				=	0.0
				,	CurrentVelocity				=	0.0								
				,	MaxVelocity					=	0.0									
				,	MaxAcceleration				=	0.0								
				,	TargetPosition				=	0.0
				,	ExecutionTimeForCurrentDOF	=	0.0
				,	MinimumExecutionTime		=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if ((*IP.SelectionVector)[i])
		{
			CurrentPosition				=	(*IP.CurrentPosition	)[i];
			CurrentVelocity				=	(*IP.CurrentVelocity	)[i];
			MaxVelocity					=	(*IP.MaxVelocity		)[i];
			MaxAcceleration				=	(*IP.MaxAcceleration	)[i];
			TargetPosition				=	(*IP.TargetPosition		)[i];
			ExecutionTimeForCurrentDOF	=	0.0;

			if (!TypeIOTGMath::Decision_1001(CurrentVelocity))
			{
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	- CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;
			}

			if (!TypeIOTGMath::Decision_1002(		CurrentVelocity
												,	MaxVelocity))
			{
				// v --> vmax
				ExecutionTimeForCurrentDOF	+=	(CurrentVelocity - MaxVelocity) / MaxAcceleration;
				CurrentPosition				+=	0.5 * (pow2(CurrentVelocity) - pow2(MaxVelocity))
												/ MaxAcceleration;
				CurrentVelocity				=	MaxVelocity;
			}

			if (!TypeIOTGMath::Decision_1003(		CurrentPosition
												,	CurrentVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{
				// v --> 0
				ExecutionTimeForCurrentDOF	+=	CurrentVelocity / MaxAcceleration;
				CurrentPosition				+=	0.5 * pow2(CurrentVelocity) / MaxAcceleration;
				CurrentVelocity				=	0.0;

				// switch signs
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;
				TargetPosition		=	-TargetPosition		;

			}

			if (TypeIOTGMath::Decision_1004(		CurrentPosition
												,	CurrentVelocity
												,	MaxVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{
				ExecutionTimeForCurrentDOF +=	TypeIOTGMath::ProfileStep1PosTrap(		CurrentPosition
																					,	CurrentVelocity
																					,	MaxVelocity
																					,	MaxAcceleration
																					,	TargetPosition);
			}
			else
			{
				ExecutionTimeForCurrentDOF +=	TypeIOTGMath::ProfileStep1PosTri(		CurrentPosition
																					,	CurrentVelocity
																					,	MaxAcceleration
																					,	TargetPosition);
			}
		}

		if (ExecutionTimeForCurrentDOF > MinimumExecutionTime)
		{
			MinimumExecutionTime = ExecutionTimeForCurrentDOF;
		}
	}

	return (MinimumExecutionTime);
}


//************************************************************************************
// SynchronizeTrajectory()

void TypeIOTG::SynchronizeTrajectory(		const TypeIOTGInputParameters	&IP
										,	const double					&SynchronizationTime
										,	TypeIMotionPolynomials			*PolynomialArray)
{
	bool			SignsWereSwitched			=	false;

	int				i							=	0;

	double			CurrentPosition				=	0.0
				,	CurrentVelocity				=	0.0								
				,	MaxVelocity					=	0.0									
				,	MaxAcceleration				=	0.0								
				,	TargetPosition				=	0.0
				,	ElapsedTime					=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if ((*IP.SelectionVector)[i])
		{
			CurrentPosition						=	(*IP.CurrentPosition	)[i];
			CurrentVelocity						=	(*IP.CurrentVelocity	)[i];
			MaxVelocity							=	(*IP.MaxVelocity		)[i];
			MaxAcceleration						=	(*IP.MaxAcceleration	)[i];
			TargetPosition						=	(*IP.TargetPosition		)[i];

			SignsWereSwitched					=	false;
			ElapsedTime							=	0.0;

			Polynomials[i].ValidPolynomials	=	0;

			if (!TypeIOTGMath::Decision_2001(CurrentVelocity))
			{
				SignsWereSwitched	=	!SignsWereSwitched	;
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;
			}

			if (!TypeIOTGMath::Decision_2002(		CurrentVelocity
												,	MaxVelocity))
			{
				TypeIOTGMath::ProfileStep2V_To_Vmax(	&CurrentPosition									
													,	&CurrentVelocity
													,	MaxVelocity
													,	MaxAcceleration
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));
			}

			if (TypeIOTGMath::Decision_2003(		CurrentPosition
												,	CurrentVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{

				if (TypeIOTGMath::Decision_2004(		CurrentPosition
													,	CurrentVelocity
													,	MaxAcceleration
													,	TargetPosition
													,	SynchronizationTime))
				{
					TypeIOTGMath::ProfileStep2PosTrap(		&CurrentPosition
														,	&CurrentVelocity
														,	MaxAcceleration
														,	TargetPosition
														,	SynchronizationTime
														,	&ElapsedTime					
														,	SignsWereSwitched
														,	&(PolynomialArray[i]));

				}
				else
				{
					TypeIOTGMath::ProfileStep2NegHldNegLin(		&CurrentPosition
															,	&CurrentVelocity
															,	MaxAcceleration
															,	TargetPosition
															,	SynchronizationTime
															,	&ElapsedTime					
															,	SignsWereSwitched
															,	&(PolynomialArray[i]));
				}
			}
			else
			{
				TypeIOTGMath::ProfileStep2V_To_Zero(	&CurrentPosition									
													,	&CurrentVelocity
													,	MaxAcceleration
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));

				// Switch signs
				SignsWereSwitched	=	!SignsWereSwitched	;
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;

				TypeIOTGMath::ProfileStep2PosTrap(		&CurrentPosition
													,	&CurrentVelocity
													,	MaxAcceleration
													,	TargetPosition
													,	SynchronizationTime
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));
			}
		}
	}
				
	return;
}


//************************************************************************************
// CalculateOutputValues()

int TypeIOTG::CalculateOutputValues(		const TypeIMotionPolynomials	*PolynomialArray
										,	const TypeIOTGBoolVector		&SelectionVector
										,	TypeIOTGOutputParameters		*OP)
{
	int				i							=	0
				,	SegmentCounter				=	0
				,	ReturnValue					=	TypeIOTG::OTG_FINAL_STATE_REACHED;


	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		SegmentCounter = 0;

		if (SelectionVector[i])
		{
			while (this->CycleTime >= PolynomialArray[i].PolynomialTimes[SegmentCounter])
			{
				SegmentCounter++;
				if (SegmentCounter > PolynomialArray[i].ValidPolynomials)
				{
					return(TypeIOTG::OTG_ERROR);
				}
			}
			
			(*OP->NewPosition)[i] = PolynomialArray[i].PositionPolynomial[SegmentCounter].CalculateValue(this->CycleTime);
			(*OP->NewVelocity)[i] = PolynomialArray[i].VelocityPolynomial[SegmentCounter].CalculateValue(this->CycleTime);


			if (SegmentCounter + 1 < PolynomialArray[i].ValidPolynomials)
			{
				ReturnValue = TypeIOTG::OTG_WORKING;
			}
		}
	}

	return (ReturnValue);
}


//************************************************************************************
// GetNextMotionState_Velocity

int TypeIOTG::GetNextMotionState_Velocity(		const double*	CurrentPosition
											,	const double*	CurrentVelocity
											,	const double*	MaxAcceleration
											,	const double*	TargetVelocity
											,	const bool*		SelectionVector
											,	double*			NewPosition
											,	double*			NewVelocity	)
{
	int				i							=	0
				,	ReturnValue					=	TypeIOTG::OTG_FINAL_STATE_REACHED;

	double			MinimumExecutionTime		=	0.0;

	this->TrajectoryExecutionTimeForTheUser		=	0.0;
	
	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if (SelectionVector[i])
		{
			if (MaxAcceleration[i] < OTG_MIN_VALUE_FOR_MAXACCELERATION)
			{
				return (TypeIOTG::OTG_MAX_ACCELERATION_ERROR);
			}

			MinimumExecutionTime	=	fabs(CurrentVelocity[i] - TargetVelocity[i]) / MaxAcceleration[i];

			if (MinimumExecutionTime > this->TrajectoryExecutionTimeForTheUser)
			{
				this->TrajectoryExecutionTimeForTheUser = MinimumExecutionTime;
			}

			if (MinimumExecutionTime > this->CycleTime)
			{
				if (CurrentVelocity[i] > TargetVelocity[i])
				{
					NewVelocity[i]	=	CurrentVelocity[i] - MaxAcceleration[i] * this->CycleTime;
				}
				else
				{
					NewVelocity[i]	=	CurrentVelocity[i] + MaxAcceleration[i] * this->CycleTime;						
				}
				NewPosition[i]	=	CurrentPosition[i] + 0.5 * (NewVelocity[i] + CurrentVelocity[i]) * this->CycleTime;

				ReturnValue		=	TypeIOTG::OTG_WORKING;
			}
			else
			{
				NewVelocity[i]	=	TargetVelocity[i];
				NewPosition[i]	=	CurrentPosition[i] + 0.5 * MinimumExecutionTime
									* (CurrentVelocity[i] - TargetVelocity[i])
									+ TargetVelocity[i] * this->CycleTime;
			}
		}
	}

	return(ReturnValue);
}


//*******************************************************************************************
// GetExecutionTime()

double TypeIOTG::GetExecutionTime(void) const
{
	return(TrajectoryExecutionTimeForTheUser);
}

