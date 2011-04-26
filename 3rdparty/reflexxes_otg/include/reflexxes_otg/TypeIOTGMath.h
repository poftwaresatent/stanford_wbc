//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGMath.h
//!
//! \brief
//! Header file for functions and definitions of constant values and macros
//! 
//! \details
//! Header file for definitions of constant values and macros to be used
//! for within in the library of the Type I On-Line Trajectory Algorithm.
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


#ifndef __TypeIOTGMath__
#define __TypeIOTGMath__


//*******************************************************************************************
// Include files



namespace TypeIOTGMath
{

//*******************************************************************************************
// Definitions and macros




//  ---------------------- Doxygen info ----------------------
//! \def OTG_INFINITY
//!
//! \brief
//! A value for infinity \f$ \infty = 10^{100} \f$
//  ----------------------------------------------------------
#define		OTG_INFINITY				((double)1.0e100)


//  ---------------------- Doxygen info ----------------------
//! \def OTG_DENOMINATOR_EPSILON
//!
//! \brief
//! A threshold value for zero to be used for denominators
//  ----------------------------------------------------------
#define		OTG_DENOMINATOR_EPSILON		((double)1.0e-12)


//  ---------------------- Doxygen info ----------------------
//! \def OTG_MIN_VALUE_FOR_MAXVELOCITY
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value for
//! the maximum velocity value
//  ----------------------------------------------------------
#define		OTG_MIN_VALUE_FOR_MAXVELOCITY		((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def OTG_MIN_VALUE_FOR_MAXACCELERATION
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value
//! for the maximum acceleration value
//  ----------------------------------------------------------
#define		OTG_MIN_VALUE_FOR_MAXACCELERATION	((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def pow2(A)
//!
//! \brief
//! A to the power of 2 
//!
//! \param A
//! Basis
//!
//! \return
//! Result value
//  ----------------------------------------------------------
#define pow2(A)							((A)*(A))


//  ---------------------- Doxygen info ----------------------
//! \fn double OTGSqrt(const double &Value)
//!
//! \brief Calculates the real square root of a given value
//!
//! \details
//! If the value is negative a value of zero will be returned.
//!
//! \param Value
//! Square root radicand
//!
//! \return
//! Square root value (real)
//  ----------------------------------------------------------
double OTGSqrt(const double &Value);


}	// namespace OTGMath

#endif
