//  ---------------------- Doxygen info ----------------------
//! \file TypeIOTGVector.h
//!
//! \brief
//! Header file for the dynamic vector class used for the Type I OTG algorithm
//!
//! \details
//! This file implements a minimalist version of a dynamic vector class
//! to be used for the interface of the Type I On-Line Trajectory
//! Generation algorithm. Furthermore, three type-specific versions 
//! of this class are defined:
//! 
//!  - OTGDoubleVector for \c double values,
//! 
//!	 - OTGIntVector for \c integer values, and
//! 
//!  - OTGBoolVector for \c bool values.
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


#ifndef __TypeIOTGVector__
#define __TypeIOTGVector__


#include <string.h>



namespace TypeIOTGMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIOTGVector
//!
//! \brief
//! This is a minimalist dynamic vector class implementation
//! for the On-Line Trajectory Generation algorithm
//! 
//! \warning
//! For all copy and comparison methods of this class, no check, 
//! whether two objects are of the same size, is implemented.
//! In order to safe computation time, it is assumed that the used
//! OTGVector objects are of the same size. Please ensure that only
//! OTGVector objects of equal size are used.
//! 
//! \sa typedef TypeIOTGDoubleVector
//! \sa typedef TypeIOTGIntVector
//! \sa typedef TypeIOTGBoolVector
//  ----------------------------------------------------------
template <class T = double>
class TypeIOTGVector
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const TypeIOTGVector<T>& Vector)
//! 
//! \brief
//! Copy constructor of class TypeIOTGVector
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated. 
//! 
//! \param Vector
//! Original object reference
//  ----------------------------------------------------------
	TypeIOTGVector(const TypeIOTGVector<T>& Vector)
	{
		VectorDimension		=	Vector.GetLength();
		VecData				=	new T[VectorDimension];

		*this				=	Vector;		
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const unsigned int Size)
//! 
//! \brief
//! Constructor of class TypeIOTGVector, allocates memory for a given
//! number of double elements
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Size
//! Determines the number of vector elements
//  ----------------------------------------------------------
	TypeIOTGVector(const unsigned int Size)
	{
		VectorDimension = Size;
		VecData = new T[VectorDimension];

		memset(VecData, 0x0, VectorDimension * sizeof(T));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1)
//! 
//! \brief
//! Special 2D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1)
	{
		VectorDimension = 2;
		VecData = (T*) new T[VectorDimension];
		VecData[0] = c0;
		VecData[1] = c1;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1, const T &Component2)
//! 
//! \brief
//! Special 3D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1, const T &c2)
	{
		VectorDimension = 3;
		VecData = (T*) new T[VectorDimension];
		VecData[0] = c0;
		VecData[1] = c1;
		VecData[2] = c2;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3)
//! 
//! \brief
//! Special 4D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1, const T &c2, const T &c3)
	{
		VectorDimension = 4;
		VecData = (T*) new T[VectorDimension];
		VecData[0] = c0;
		VecData[1] = c1;
		VecData[2] = c2;
		VecData[3] = c3;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4)
//! 
//! \brief Special 5D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1, const T &c2, const T &c3, const T &c4)
	{
		VectorDimension = 5;
		VecData = (T*) new T[VectorDimension];
		VecData[0] = c0;
		VecData[1] = c1;
		VecData[2] = c2;
		VecData[3] = c3;
		VecData[4] = c4;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4, const T &Component5)
//! 
//! \brief
//! Special 6D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//! 
//! \param Component5
//! Value of the sixth vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1, const T &c2, const T &c3, const T &c4, const T &c5)
	{
		VectorDimension = 6;
		//VecData = (T*) new T[VectorDimension];

		//VecData = &(MyArray[0]);
		VecData[0] = c0;
		VecData[1] = c1;
		VecData[2] = c2;
		VecData[3] = c3;
		VecData[4] = c4;
		VecData[5] = c5;
	}
	

//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector(const T &Component0, const T &Component1, const T &Component2, const T &Component3, const T &Component4, const T &Component5, const T &Component6)
//! 
//! \brief
//! Special 7D constructor
//! 
//! \warning
//! This method is \em not real-time capable as heap memory has to be
//! allocated.
//! 
//! \param Component0
//! Value of the first vector component
//! 
//! \param Component1
//! Value of the second vector component
//! 
//! \param Component2
//! Value of the third vector component
//! 
//! \param Component3
//! Value of the fourth vector component
//! 
//! \param Component4
//! Value of the fifth vector component
//! 
//! \param Component5
//! Value of the sixth vector component
//! 
//! \param Component6
//! Value of the seventh vector component
//  ----------------------------------------------------------
	TypeIOTGVector(const T &c0, const T &c1, const T &c2, const T &c3, const T &c4, const T &c5, const T &c6)
	{
		VectorDimension = 7;
		VecData = (T*) new T[VectorDimension];
		VecData[0] = c0;
		VecData[1] = c1;
		VecData[2] = c2;
		VecData[3] = c3;
		VecData[4] = c4;
		VecData[5] = c5;
		VecData[6] = c6;
	}
	

//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIOTGVector(void)
//! 
//! \brief
//! Destructor of class TypeIOTGVector
//  ----------------------------------------------------------
	~TypeIOTGVector(void)
	{
		delete[] VecData;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn void Set(const T Value)
//! 
//! \brief Sets all elements of a vector of double elements to one specific value
//! 
//! \param Value
//! Value for all elements of the vector
//  ----------------------------------------------------------
	void Set(const T Value)
	{
		unsigned int i;

		for( i = 0; i < VectorDimension; i++)
		{
			VecData[i] = Value;
		}
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIOTGVector &operator = (const TypeIOTGVector<T>& Vector)
//! 
//! \brief
//! Copy operator
//! 
//! \param Vector
//! Vector object to be copied
//  ----------------------------------------------------------
	TypeIOTGVector &operator = (const TypeIOTGVector<T>& Vector)
	{
		memcpy(this->VecData, Vector.GetReference(), (this->VectorDimension * sizeof(T)));
		return(*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn T& operator [] (const int Index)
//! 
//! \brief
//! Bracket operator, gives access to a single vector element
//! 
//! \param Index
//! Determines the desired vector element
//! 
//! \return
//! Reference to one single vector element
//  ----------------------------------------------------------
	T& operator [] (const int Index)
	{
		return (VecData[Index]);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn const T& operator [] (const int Index) const
//! 
//! \brief
//! Bracket operator, gives access to a single vector element
//! 
//! \param Index
//! Determines the desired vector element
//! 
//! \return
//! Constant pointer to one single vector element
//  ----------------------------------------------------------
	const T& operator [] (const int Index) const
	{
		return(VecData[Index]);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn bool operator == (const TypeIOTGVector<T> &Vector) const
//! 
//! \brief
//! Equal operator
//! 
//! \return
//! \c true if all vector elements are equal or \c false in all other cases
//  ----------------------------------------------------------
	bool operator == (const TypeIOTGVector<T> &Vector) const
	{
		unsigned int i;

		for( i = 0; i < VectorDimension; i++)
		{
			if( (*this)[i] != Vector[i] )
			{
				return(false); // components !=
			}
		}
		return(true); // all components ==
	}


//  ---------------------- Doxygen info ----------------------
//! \fn bool operator != (const TypeIOTGVector<T> &Vector) const
//! 
//! \brief
//! Unequal operator
//! \return
//! \c true if all vector elements are equal and \c false in all other cases
//  ----------------------------------------------------------
	bool operator != (const TypeIOTGVector<T> &Vector) const
	{
		return(!(*this == Vector));
	}
	
	
//  ---------------------- Doxygen info ----------------------
//! \fn int GetLength(void) const
//! 
//! \brief
//! Returns the length of the vector
//! 
//! \return
//! Number of vector elements
//  ----------------------------------------------------------
	int GetLength(void) const
	{
		return (this->VectorDimension);
	}

//  ---------------------- Doxygen info ----------------------
//! \fn T* GetReference(void) const
//! 
//! \brief
//! Returns the \em data pointer of the vector object (not the pointer to the \em object)
//! 
//! \return
//! Data pointer
//  ----------------------------------------------------------
	T* GetReference(void) const
	{
		return((T*)VecData);
	}
	

private:

	T				*VecData;
	unsigned int	VectorDimension;



};	// class TypeIOTGVector


//  ---------------------- Doxygen info ----------------------
//! \typedef TypeIOTGDoubleVector
//!
//! \brief
//! Type definition for vectors of \c double elements
//!
//! \sa class TypeIOTGVector
//  ----------------------------------------------------------
typedef	TypeIOTGVector<double>				TypeIOTGDoubleVector;


//  ---------------------- Doxygen info ----------------------
//! \typedef TypeIOTGIntVector
//!
//! \brief
//! Type definition for vectors of \c int elements
//!
//! \sa class TypeIOTGVector
//  ----------------------------------------------------------
typedef	TypeIOTGVector<int>					TypeIOTGIntVector;


//  ---------------------- Doxygen info ----------------------
//! \typedef TypeIOTGBoolVector
//!
//! \brief
//! Type definition for vectors of \c bool elements
//!
//! \sa class TypeIOTGVector
//  ----------------------------------------------------------
typedef	TypeIOTGVector<bool>				TypeIOTGBoolVector;

}	// namespace TypeIOTGMath

#endif
