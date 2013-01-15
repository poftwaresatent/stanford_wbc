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

#ifndef _deMassProp_h
#define _deMassProp_h

#include <tao/matrix/TaoDeTypes.h>
#include <tao/matrix/TaoDeMath.h>

/*!
 *	\ingroup deUtility
 *	\name Density
 *	densities for common materials in SI unit (- kg/m^3)
 */
//	@{
#define DE_DENSITY_AIR				(-1.2062f)
#define DE_DENSITY_ALUMINUM			(-2690.0f)
#define DE_DENSITY_CONCRETE			(-2400.0f)
#define DE_DENSITY_COPPER			(-8910.0f)
#define DE_DENSITY_EARTH_WET		(-1760.0f)
#define DE_DENSITY_EARTH_DRY		(-1280.0f)
#define DE_DENSITY_GLASS			(-2590.0f)
#define DE_DENSITY_GOLD				(-19300.0f)
#define DE_DENSITY_ICE				(-900.0f)
#define DE_DENSITY_IRON				(-7210.0f)
#define DE_DENSITY_LEAD				(-11370.0f)
#define DE_DENSITY_MERCURY			(-13570.0f)
#define DE_DENSITY_OIL				(-900.0f)
#define DE_DENSITY_STEEL			(-7830.0f)
#define DE_DENSITY_TITANIUM			(-3080.0f)
#define DE_DENSITY_WATER			(-1000.0f)
#define DE_DENSITY_WATER_SALT		(-1030.0f)
#define DE_DENSITY_WOOD_SOFTPINE	(-480.0f)
#define DE_DENSITY_WOOD_HARDOAK		(-800.0f)
//	@}

/*!
 *	\brief		Compute Mass parameters
 *	\ingroup	deUtility
 */
class deMassProp
{
  public:
    //! constructor
    inline deMassProp() { zero(); }
  
    inline deMassProp(deMassProp const & orig) {
      _m = orig._m;
      _center = orig._center;
      _inertia = orig._inertia;
    }
  
    //! set members to zero
    void zero() {
      _m = 0;
      _center.zero();
      _inertia.zero();
    }
    //! set members with given values
    void set(const deFloat* mass, const deVector3* center, const deMatrix3* inertia)
    {
      _m = *mass;
      _center = *center;
      _inertia = *inertia;
    }
    //! get member values
    void get(deFloat* mass, deVector3* center, deMatrix3* inertia) const
    {
      *mass = _m;
      *center = _center;
      *inertia = _inertia;
    }
    /*!
     *	\brief	indicates if \a m is a density.
     *	\remarks	negative \a m indicates \a m is a density.
     *	\retval	0 if \a m represents a mass
     *	\retval	1 if \a m represents a density
     */
    deInt isDensity(deFloat m) { return (m < 0); }

    /*! @name fetching methods
     * return current mass parameters.
     */

    /** \return mass of object. */
    deFloat* mass() { return &_m; }
    deFloat const * mass() const { return &_m; }
  
    /** \return inertia of object as seen from objects local frame. */
    deMatrix3* inertia() { return &_inertia; }
    deMatrix3 const * inertia() const { return &_inertia; }

    /** \return center of mass of object as seen from objects local frame. */
    deVector3* center() { return &_center; }
    deVector3 const * center() const { return &_center; }

    /*!	@name updating methods
     *	set general mass/inertia parameters for a body defined in the current reference frame.
     *	\remarks Calls are accumulative.
     */

    void mass(const deFloat m, const deFrame* f = NULL);
    /*!< add a point mass of mass \a m at a point at the origin of the reference frame.
     *    Set frame of reference to be used when specifying a objects mass/inertial properties.
     *  \param m mass to be added
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void inertia(const deMatrix3* inertia, const deFrame* f = NULL);
    /*!< add an inertia tensor \a inertia specified by a 3x3 matrix at the current frame of reference. */

    void inertia(const deVector3* diag, const deFrame* f = NULL); 
    /*!< add an inertia tensor specified by its diagonal \a diag = (\a Ixx, \a Iyy, \a Izz). */

    void inertia(const deFloat Ixx,const deFloat Iyy,const deFloat Izz, const deFrame* f = NULL);
    /*!< add an inertia tensor specified by its diagonal (\a Ixx, \a Iyy, \a Izz). */

    /*! @name mass/inertia definitions of homogeneous bodies
     *	specify mass parameters for various homogeneous bodies. With the center of mass located
     *	at the current reference frame.
     *	\remarks Calls are accumulative.
     *	\param f new frame of reference. \a NULL indicates the identity matrix..
     */

    void cylinder(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous cylinder with center
     *  of mass at the current frame.
     *  \param mp mass (density if dedensity(deFloat)) of cylinder.
     *  \param h  total height of cylinder in z-axis.
     *  \param r  radius of the cylinder.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void cone(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous cone with center
     *  of mass at the current frame. The center of mass of a cone is
     *  located \a h/4 from base in \a z
     *  \param mp mass (density if dedensity(deFloat)) of cone.
     *  \param h  total height of cone in z-axis.
     *  \param r  radius of the base of the cone.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void pyramid(const deFloat mp, const deFloat a, const deFloat b, const deFloat h, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous four sided pyramid with
     *  center of mass at the current frame. The center of mass of a pyramid
     *  is located \a h/4 from base in \a z
     *  \param mp mass (density if dedensity(deFloat)) of pyramid.
     *  \param a  width of base of pyramid along the x-axis.
     *  \param b  length of base of pyramid along the y-axis.
     *  \param h  total height of pyramid h in z-axis.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void block(const deFloat mp, const deFloat a, const deFloat b, const deFloat c, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous block with center of
     *  mass at the current frame.
     *  \param mp mass (density if dedensity(deFloat)) of block.
     *  \param a  width of base of block along the x-axis.
     *  \param b  length of base of block along the y-axis.
     *  \param c  height of base of block along the z-axis.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void sphere(const deFloat mp, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous sphere with center of
     *  mass at the current frame.
     *  \param mp mass (density if dedensity(deFloat)) of sphere.
     *  \param r  radius of sphere.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void hemisphere(const deFloat mp, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous half sphere (hemisphere) with
     *  center of mass at the current frame. The center of mass of a hemisphere
     *  is located \a (3/8)*r from base in \a z
     *  \param mp mass (density if dedensity(deFloat)) of hemisphere.
     *  \param r  radius of hemisphere.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void ellipsoid(const deFloat mp, const deFloat a, const deFloat b, const deFloat c, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous ellipsoid with center of
     *  mass at the current frame.
     *  \param mp mass (density if dedensity(deFloat)) of ellipsoid.
     *  \param a  length of major axis along the x-axis, one half total width
     *  \param b  length of major axis along the y-axis, one half total length
     *  \param c  length of major axis along the z-axis, one half total height
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void rod(const deFloat mp, const deFloat l, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous one dimensional rod having
     *  total length \a l with center of mass at the current frame.
     *  The center of mass of a rod is located half way along its length.
     *  \param mp mass (linear density if dedensity(deFloat)).
     *  \param l  total length of the rod along the z-axis.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void disk(const deFloat mp, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous two dimensional flat disk
     *  with center of mass at the current frame. The disk has zero height in
     *  the z-axis.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param r  radius of disk.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void plate(const deFloat mp, const deFloat a, const deFloat b, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous two dimensional flat square
     *  plate with center of mass at the current frame. The plate has zero height in
     *  the z-axis.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param a  total width of the in the x-axis .
     *  \param b  total width of the in the y-axis .
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void cylinderShell(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous infinitely
     *  thin cylinder with no caps, with center of mass at the current frame.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param h  total height of cylinder shell in z-axis.
     *  \param r  radius of the cylinder shell.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void coneShell(const deFloat mp, const deFloat h, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous infinitely
     *  thin cone with no base. The center of mass located at the current frame
     *  is \a (1/3)*h from base in \a z.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param h  total height of cone shell h in z-axis.
     *  \param r  radius of the base of the cone shell.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void sphereShell(const deFloat mp, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous infinitely
     *  thin sphere, with center of mass at the current frame.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param r  radius of the sphere.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    void hemisphereShell(const deFloat mp, const deFloat r, const deFrame* f = NULL);
    /*!< 
     *  Specify the mass parameters to a homogeneous infinitely
     *  thin half sphere (hemisphere) with no base. The center of mass
     *  located at the current frame is r/2 from the base of the hemisphere in
     *  the z-axis.
     *  \param mp mass (surface density if dedensity(deFloat)).
     *  \param r  radius of the hemisphere.
     *	\param f new frame of reference. \a NULL (default) indicates the identity matrix.
     */

    /*! @name resetting methods
     *	reset mass properties
     */
    void scale(const deFloat m); 
    /*!< homogenously scale the mass properities of a given object until total mass equals \a m. */

    /*! @name used to keep doxygen happy */       
    //@{
    //@}
  private:
    deFloat _m;
    deVector3 _center;
    deMatrix3 _inertia;
};
#endif
