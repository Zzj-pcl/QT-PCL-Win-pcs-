#ifndef PCSBBOX_H
#define PCSBBOX_H

#include "pcs_pc_lib_global.h"
#include "pcsGeometry.h"

class PCS_PC_LIB_EXPORT pcsBBox
{
public:
	//! Default constructor
	pcsBBox();
	//! Copy constructor
	pcsBBox(const pcsBBox& aBBox);
	//! Constructor from two vectors (lower min. and upper max. corners)
	pcsBBox(const pcsVector3d& bbMinCorner, const pcsVector3d& bbMaxCorner);

	//! Returns the 'sum' of this bounding-box and another one
	pcsBBox operator + (const pcsBBox& aBBox) const;
	//! In place 'sum' of this bounding-box with another one
	const pcsBBox& operator += (const pcsBBox& aBBox);
	//! Shifts the bounding box with a vector
	const pcsBBox& operator += (const pcsVector3d& aVector);
	//! Shifts the bounding box with a vector
	const pcsBBox& operator -= (const pcsVector3d& aVector);
	//! Scales the bounding box
	const pcsBBox& operator *= (PointCoordinateType scaleFactor);
	////! Rotates the bounding box
	//const pcsBBox& operator *= (const pcsLib::SquareMatrix& aMatrix);
	////! Applies transformation to the bounding box
	//const pcsBBox operator * (const pcsGLMatrix& mat);
	////! Applies transformation to the bounding box
	//const pcsBBox operator * (const pcsGLMatrixd& mat);

	//! Resets the bounding box
	/** (0,0,0) --> (0,0,0)
	**/
	void clear();

	//! 'Enlarges' the bounding box with a point
	void add(const pcsVector3d& aPoint);

	//! Returns min corner (const)
	inline const pcsVector3d& minCorner() const { return m_bbMin; }
	//! Returns max corner (const)
	inline const pcsVector3d& maxCorner() const { return m_bbMax; }

	//! Returns min corner
	inline pcsVector3d& minCorner() { return m_bbMin; }
	//! Returns max corner
	inline pcsVector3d& maxCorner() { return m_bbMax; }

	//! Returns center
	pcsVector3d getCenter() const;
	//! Returns diagonal vector
	pcsVector3d getDiagVec() const;
	//! Returns diagonal length
	inline PointCoordinateType getDiagNorm() const { return getDiagVec().norm(); }
	//! Returns diagonal length (double precision)
	double getDiagNormd() const { return getDiagVec().normd(); }
	//! Returns minimal box dimension
	PointCoordinateType getMinBoxDim() const;
	//! Returns maximal box dimension
	PointCoordinateType getMaxBoxDim() const;

	//! Draws bounding box (OpenGL)
	/** \param col (R,G,B) color
	**/
	/*void draw(const colorType col[]) const;*/

	//! Sets bonding box validity
	inline void setValidity(bool state) { m_valid = state; }

	//! Returns whether bounding box is valid or not
	inline bool isValid() const { return m_valid; }

	//! Computes min gap (absolute distance) between this bounding-box and another one
	/** \return min gap (>=0) or -1 if at least one of the box is not valid
	**/
	PointCoordinateType minDistTo(const pcsBBox& box) const;

	//! Returns whether a points is inside the box or not
	/** Warning: box should be valid!
	**/
	inline bool contains(const pcsVector3d& P) const
	{
		return (P.x >= m_bbMin.x && P.x <= m_bbMax.x &&
			P.y >= m_bbMin.y && P.y <= m_bbMax.y &&
			P.z >= m_bbMin.z && P.z <= m_bbMax.z);
	}

protected:

	//! Lower min. corner
	pcsVector3d m_bbMin;
	//! Upper max. corner
	pcsVector3d m_bbMax;
	//! Validity
	bool m_valid;
	
};

#endif // PCSBBOX_H
