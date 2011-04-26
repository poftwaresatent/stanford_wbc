//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGDecision.h
//!
//! \brief
//! Header file for decisions of the two decision trees of the 
//! Type I On-Line Trajectory Generation algorithm
//!
//! \details
//! This file contains all neccessary decisions for the Type I On-Line
//! Trajectory Generation algorithm. All functions are part
//! of the namespace TypeIOTGMath.
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


#ifndef __TypeIOTGDecision__
#define __TypeIOTGDecision__



namespace TypeIOTGMath
{	

//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_1001(		const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi >= vmax)?
//  ----------------------------------------------------------
//is (vi <= vmax)?
bool Decision_1002(		const double &CurrentVelocity
					,	const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1003(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->0, is (p<=ptrgt)?
//  ----------------------------------------------------------
bool Decision_1003(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1004(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->+vmax->0, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1004(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_2001(		const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= vmax)?
//  ----------------------------------------------------------
bool Decision_2002(		const double &CurrentVelocity
					,	const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2003(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->0, is (p<=ptrgt)?
//  ----------------------------------------------------------
bool Decision_2003(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2004(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime)
//!
//! \brief
//! If v->hold->0, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2004(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition
					,	const double &SynchronizationTime);

}	// namespace TypeIOTGMath

#endif
