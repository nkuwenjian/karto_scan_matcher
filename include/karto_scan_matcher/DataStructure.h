#ifndef _DATA_STRUCTURE_H_
#define _DATA_STRUCTURE_H_

#include <karto_scan_matcher/Math.h>
#include <karto_scan_matcher/Macros.h>

#include <vector>
#include <string.h>

namespace KartoScanMatcher
{
/**
 * Represents a size (width, height) in 2-dimensional real space.
 */
template<typename T>
class Size2
{
public:
	/**
	 * Default constructor
	 */
	Size2()
	: m_Width(0)
	, m_Height(0)
	{
	}

	/**
	 * Constructor initializing point location
	 * @param width
	 * @param height
	 */
	Size2(T width, T height)
	: m_Width(width)
	, m_Height(height)
	{
	}

	/**
	 * Copy constructor
	 * @param rOther
	 */
	Size2(const Size2& rOther)
	: m_Width(rOther.m_Width)
	, m_Height(rOther.m_Height)
	{
	}

public:
	/**
	 * Gets the width
	 * @return the width
	 */
	inline const T GetWidth() const
	{
		return m_Width;
	}

	/**
	 * Sets the width
	 * @param width
	 */
	inline void SetWidth(T width)
	{
		m_Width = width;
	}

	/**
	 * Gets the height
	 * @return the height
	 */
	inline const T GetHeight() const
	{
		return m_Height;
	}

	/**
	 * Sets the height
	 * @param height
	 */
	inline void SetHeight(T height)
	{
		m_Height = height;
	}

	/**
	 * Assignment operator
	 */
	inline Size2& operator = (const Size2& rOther)
	{
		m_Width = rOther.m_Width;
		m_Height = rOther.m_Height;

		return(*this);
	}

	/**
	 * Equality operator
	 */
	inline kt_bool operator == (const Size2& rOther) const
	{
		return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
	}

	/**
	 * Inequality operator
	 */
	inline kt_bool operator != (const Size2& rOther) const
	{
		return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
	}

private:
	T m_Width;
	T m_Height;
};  // Size2<T>

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Represents a vector (x, y) in 2-dimensional real space.
 */
template<typename T>
class Vector2
{
public:
	/**
	 * Default constructor
	 */
	Vector2()
	{
		m_Values[0] = 0;
		m_Values[1] = 0;
	}

	/**
	 * Constructor initializing vector location
	 * @param x
	 * @param y
	 */
	Vector2(T x, T y)
	{
		m_Values[0] = x;
		m_Values[1] = y;
	}

public:
	/**
	 * Gets the x-coordinate of this vector2
	 * @return the x-coordinate of the vector2
	 */
	inline const T& GetX() const
	{
		return m_Values[0];
	}

	/**
	 * Sets the x-coordinate of this vector2
	 * @param x the x-coordinate of the vector2
	 */
	inline void SetX(const T& x)
	{
		m_Values[0] = x;
	}

	/**
	 * Gets the y-coordinate of this vector2
	 * @return the y-coordinate of the vector2
	 */
	inline const T& GetY() const
	{
		return m_Values[1];
	}

	/**
	 * Sets the y-coordinate of this vector2
	 * @param y the y-coordinate of the vector2
	 */
	inline void SetY(const T& y)
	{
		m_Values[1] = y;
	}

	/**
	 * Floor point operator
	 * @param rOther
	 */
	inline void MakeFloor(const Vector2& rOther)
	{
		if ( rOther.m_Values[0] < m_Values[0] ) m_Values[0] = rOther.m_Values[0];
		if ( rOther.m_Values[1] < m_Values[1] ) m_Values[1] = rOther.m_Values[1];
	}

	/**
	 * Ceiling point operator
	 * @param rOther
	 */
	inline void MakeCeil(const Vector2& rOther)
	{
		if ( rOther.m_Values[0] > m_Values[0] ) m_Values[0] = rOther.m_Values[0];
		if ( rOther.m_Values[1] > m_Values[1] ) m_Values[1] = rOther.m_Values[1];
	}

	/**
	 * Returns the square of the length of the vector
	 * @return square of the length of the vector
	 */
	inline kt_double SquaredLength() const
	{
		return math::Square(m_Values[0]) + math::Square(m_Values[1]);
	}

	/**
	 * Returns the length of the vector (x and y).
	 * @return length of the vector
	 */
	inline kt_double Length() const
	{
		return sqrt(SquaredLength());
	}

	/**
	 * Returns the square distance to the given vector
	 * @returns square distance to the given vector
	 */
	inline kt_double SquaredDistance(const Vector2& rOther) const
	{
		return (*this - rOther).SquaredLength();
	}

	/**
	 * Gets the distance to the other vector2
	 * @param rOther
	 * @return distance to other vector2
	 */
	inline kt_double Distance(const Vector2& rOther) const
	{
		return sqrt(SquaredDistance(rOther));
	}

public:
	/**
	 * In place Vector2 addition.
	 */
	inline void operator += (const Vector2& rOther)
	{
		m_Values[0] += rOther.m_Values[0];
		m_Values[1] += rOther.m_Values[1];
	}

	/**
	 * In place Vector2 subtraction.
	 */
	inline void operator -= (const Vector2& rOther)
	{
		m_Values[0] -= rOther.m_Values[0];
		m_Values[1] -= rOther.m_Values[1];
	}

	/**
	 * Addition operator
	 * @param rOther
	 * @return vector resulting from adding this vector with the given vector
	 */
	inline const Vector2 operator + (const Vector2& rOther) const
	{
		return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
	}

	/**
	 * Subtraction operator
	 * @param rOther
	 * @return vector resulting from subtracting this vector from the given vector
	 */
	inline const Vector2 operator - (const Vector2& rOther) const
	{
		return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
	}

	/**
	 * In place scalar division operator
	 * @param scalar
	 */
	inline void operator /= (T scalar)
	{
		m_Values[0] /= scalar;
		m_Values[1] /= scalar;
	}

	/**
	 * Divides a Vector2
	 * @param scalar
	 * @return scalar product
	 */
	inline const Vector2 operator / (T scalar) const
	{
		return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
	}

	/**
	 * Computes the dot product between the two vectors
	 * @param rOther
	 * @return dot product
	 */
	inline kt_double operator * (const Vector2& rOther) const
	{
		return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
	}

	/**
	 * Scales the vector by the given scalar
	 * @param scalar
	 */
	inline const Vector2 operator * (T scalar) const
	{
		return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
	}

	/**
	 * Subtract the vector by the given scalar
	 * @param scalar
	 */
	inline const Vector2 operator - (T scalar) const
	{
		return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
	}

	/**
	 * In place scalar multiplication operator
	 * @param scalar
	 */
	inline void operator *= (T scalar)
	{
		m_Values[0] *= scalar;
		m_Values[1] *= scalar;
	}

	/**
	 * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
	 * @param rOther
	 */
	inline kt_bool operator == (const Vector2& rOther) const
	{
		return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
	}

	/**
	 * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
	 * @param rOther
	 */
	inline kt_bool operator != (const Vector2& rOther) const
	{
		return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
	}

	/**
	 * Less than operator
	 * @param rOther
	 * @return true if left vector is less than right vector
	 */
	inline kt_bool operator < (const Vector2& rOther) const
	{
		if (m_Values[0] < rOther.m_Values[0])
			return true;
		else if (m_Values[0] > rOther.m_Values[0])
			return false;
		else
			return (m_Values[1] < rOther.m_Values[1]);
	}

private:
	T m_Values[2];
};  // Vector2<T>

/**
 * Type declaration of Vector2<kt_double> vector
 */
typedef std::vector< Vector2<kt_double> > PointVectorDouble;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Stores x, y, width and height that represents the location and size of a rectangle
 * (x, y) is at bottom left in mapper!
 */
template<typename T>
class Rectangle2
{
public:
	/**
	 * Default constructor
	 */
	Rectangle2()
	{
	}

	/**
	 * Constructor initializing rectangle parameters
	 * @param x x-coordinate of left edge of rectangle
	 * @param y y-coordinate of bottom edge of rectangle
	 * @param width width of rectangle
	 * @param height height of rectangle
	 */
	Rectangle2(T x, T y, T width, T height)
	: m_Position(x, y)
	, m_Size(width, height)
	{
	}

	/**
	 * Constructor initializing rectangle parameters
	 * @param rPosition (x,y)-coordinate of rectangle
	 * @param rSize Size of the rectangle
	 */
	Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
	: m_Position(rPosition)
	, m_Size(rSize)
	{
	}

	/**
	 * Copy constructor
	 */
	Rectangle2(const Rectangle2& rOther)
	: m_Position(rOther.m_Position)
	, m_Size(rOther.m_Size)
	{
	}

public:
	/**
	 * Gets the x-coordinate of the left edge of this rectangle
	 * @return the x-coordinate of the left edge of this rectangle
	 */
	inline T GetX() const
	{
		return m_Position.GetX();
	}

	/**
	 * Sets the x-coordinate of the left edge of this rectangle
	 * @param x the x-coordinate of the left edge of this rectangle
	 */
	inline void SetX(T x)
	{
		m_Position.SetX(x);
	}

	/**
	 * Gets the y-coordinate of the bottom edge of this rectangle
	 * @return the y-coordinate of the bottom edge of this rectangle
	 */
	inline T GetY() const
	{
		return m_Position.GetY();
	}

	/**
	 * Sets the y-coordinate of the bottom edge of this rectangle
	 * @param y the y-coordinate of the bottom edge of this rectangle
	 */
	inline void SetY(T y)
	{
		m_Position.SetY(y);
	}

	/**
	 * Gets the width of this rectangle
	 * @return the width of this rectangle
	 */
	inline T GetWidth() const
	{
		return m_Size.GetWidth();
	}

	/**
	 * Sets the width of this rectangle
	 * @param width the width of this rectangle
	 */
	inline void SetWidth(T width)
	{
		m_Size.SetWidth(width);
	}

	/**
	 * Gets the height of this rectangle
	 * @return the height of this rectangle
	 */
	inline T GetHeight() const
	{
		return m_Size.GetHeight();
	}

	/**
	 * Sets the height of this rectangle
	 * @param height the height of this rectangle
	 */
	inline void SetHeight(T height)
	{
		m_Size.SetHeight(height);
	}

	/**
	 * Gets the position of this rectangle
	 * @return the position of this rectangle
	 */
	inline const Vector2<T>& GetPosition() const
	{
		return m_Position;
	}

	/**
	 * Sets the position of this rectangle
	 * @param rX x
	 * @param rY y
	 */
	inline void SetPosition(const T& rX, const T& rY)
	{
		m_Position = Vector2<T>(rX, rY);
	}

	/**
	 * Sets the position of this rectangle
	 * @param rPosition position
	 */
	inline void SetPosition(const Vector2<T>& rPosition)
	{
		m_Position = rPosition;
	}

	/**
	 * Gets the size of this rectangle
	 * @return the size of this rectangle
	 */
	inline const Size2<T>& GetSize() const
	{
		return m_Size;
	}

	/**
	 * Sets the size of this rectangle
	 * @param rSize size
	 */
	inline void SetSize(const Size2<T>& rSize)
	{
		m_Size = rSize;
	}

	/**
	 * Gets the center of this rectangle
	 * @return the center of this rectangle
	 */
	inline const Vector2<T> GetCenter() const
	{
		return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5, m_Position.GetY() + m_Size.GetHeight() * 0.5);
	}

public:
	/**
	 * Assignment operator
	 */
	Rectangle2& operator = (const Rectangle2& rOther)
	{
		m_Position = rOther.m_Position;
		m_Size = rOther.m_Size;

		return *this;
	}

	/**
	 * Equality operator
	 */
	inline kt_bool operator == (const Rectangle2& rOther) const
	{
		return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
	}

	/**
	 * Inequality operator
	 */
	inline kt_bool operator != (const Rectangle2& rOther) const
	{
		return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
	}

private:
	Vector2<T> m_Position;
	Size2<T> m_Size;
};  // Rectangle2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2
{
public:
	/**
	 * Default Constructor
	 */
	Pose2()
	: m_Heading(0.0)
	{
	}

	/**
	 * Constructor initializing pose parameters
	 * @param rPosition position
	 * @param heading heading
	 */
	Pose2(const Vector2<kt_double>& rPosition, kt_double heading)
	: m_Position(rPosition)
	, m_Heading(heading)
	{
	}

	/**
	 * Constructor initializing pose parameters
	 * @param x x-coordinate
	 * @param y y-coordinate
	 * @param heading heading
	 */
	Pose2(kt_double x, kt_double y, kt_double heading)
	: m_Position(x, y)
	, m_Heading(heading)
	{
	}

	/**
	 * Copy constructor
	 */
	Pose2(const Pose2& rOther)
	: m_Position(rOther.m_Position)
	, m_Heading(rOther.m_Heading)
	{
	}

public:
	/**
	 * Returns the x-coordinate
	 * @return the x-coordinate of the pose
	 */
	inline kt_double GetX() const
	{
		return m_Position.GetX();
	}

	/**
	 * Sets the x-coordinate
	 * @param x the x-coordinate of the pose
	 */
	inline void SetX(kt_double x)
	{
		m_Position.SetX(x);
	}

	/**
	 * Returns the y-coordinate
	 * @return the y-coordinate of the pose
	 */
	inline kt_double GetY() const
	{
		return m_Position.GetY();
	}

	/**
	 * Sets the y-coordinate
	 * @param y the y-coordinate of the pose
	 */
	inline void SetY(kt_double y)
	{
		m_Position.SetY(y);
	}

	/**
	 * Returns the position
	 * @return the position of the pose
	 */
	inline const Vector2<kt_double>& GetPosition() const
	{
		return m_Position;
	}

	/**
	 * Sets the position
	 * @param rPosition of the pose
	 */
	inline void SetPosition(const Vector2<kt_double>& rPosition)
	{
		m_Position = rPosition;
	}

	/**
	 * Returns the heading of the pose (in radians)
	 * @return the heading of the pose
	 */
	inline kt_double GetHeading() const
	{
		return m_Heading;
	}

	/**
	 * Sets the heading
	 * @param heading of the pose
	 */
	inline void SetHeading(kt_double heading)
	{
		m_Heading = heading;
	}

	/**
	 * Return the squared distance between two Pose2
	 * @return squared distance
	 */
	inline kt_double SquaredDistance(const Pose2& rOther) const
	{
		return m_Position.SquaredDistance(rOther.m_Position);
	}

public:
	/**
	 * Assignment operator
	 */
	inline Pose2& operator = (const Pose2& rOther)
	{
		m_Position = rOther.m_Position;
		m_Heading = rOther.m_Heading;

		return *this;
	}

	/**
	 * Equality operator
	 */
	inline kt_bool operator == (const Pose2& rOther) const
	{
		return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
	}

	/**
	 * Inequality operator
	 */
	inline kt_bool operator != (const Pose2& rOther) const
	{
		return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
	}

	/**
	 * In place Pose2 add.
	 */
	inline void operator += (const Pose2& rOther)
	{
		m_Position += rOther.m_Position;
		m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
	}

	/**
	 * Binary Pose2 add
	 * @param rOther
	 * @return Pose2 sum
	 */
	inline Pose2 operator + (const Pose2& rOther) const
	{
		return Pose2(m_Position + rOther.m_Position, math::NormalizeAngle(m_Heading + rOther.m_Heading));
	}

	/**
	 * Binary Pose2 subtract
	 * @param rOther
	 * @return Pose2 difference
	 */
	inline Pose2 operator - (const Pose2& rOther) const
	{
		return Pose2(m_Position - rOther.m_Position, math::NormalizeAngle(m_Heading - rOther.m_Heading));
	}


private:
	Vector2<kt_double> m_Position;

	kt_double m_Heading;
};  // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector< Pose2 > Pose2Vector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a Matrix 3 x 3 class.
 */
class Matrix3
{
public:
	/**
	 * Default constructor
	 */
	Matrix3()
	{
		Clear();
	}

	/**
	 * Copy constructor
	 */
	inline Matrix3(const Matrix3& rOther)
	{
		memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
	}

public:
	/**
	 * Sets this matrix to identity matrix
	 */
	void SetToIdentity()
	{
		memset(m_Matrix, 0, 9*sizeof(kt_double));

		for (kt_int32s i = 0; i < 3; i++)
		{
			m_Matrix[i][i] = 1.0;
		}
	}

	/**
	 * Sets this matrix to zero matrix
	 */
	void Clear()
	{
		memset(m_Matrix, 0, 9*sizeof(kt_double));
	}

	/**
	 * Sets this matrix to be the rotation matrix of rotation around given axis
	 * @param x x-coordinate of axis
	 * @param y y-coordinate of axis
	 * @param z z-coordinate of axis
	 * @param radians amount of rotation
	 */
	void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
	{
		kt_double cosRadians = cos(radians);
		kt_double sinRadians = sin(radians);
		kt_double oneMinusCos = 1.0 - cosRadians;

		kt_double xx = x * x;
		kt_double yy = y * y;
		kt_double zz = z * z;

		kt_double xyMCos = x * y * oneMinusCos;
		kt_double xzMCos = x * z * oneMinusCos;
		kt_double yzMCos = y * z * oneMinusCos;

		kt_double xSin = x * sinRadians;
		kt_double ySin = y * sinRadians;
		kt_double zSin = z * sinRadians;

		m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
		m_Matrix[0][1] = xyMCos - zSin;
		m_Matrix[0][2] = xzMCos + ySin;

		m_Matrix[1][0] = xyMCos + zSin;
		m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
		m_Matrix[1][2] = yzMCos - xSin;

		m_Matrix[2][0] = xzMCos - ySin;
		m_Matrix[2][1] = yzMCos + xSin;
		m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
	}

	/**
	 * Returns transposed version of this matrix
	 * @return transposed matrix
	 */
	Matrix3 Transpose() const
	{
		Matrix3 transpose;

		for (kt_int32u row = 0; row < 3; row++)
		{
			for (kt_int32u col = 0; col < 3; col++)
			{
				transpose.m_Matrix[row][col] = m_Matrix[col][row];
			}
		}

		return transpose;
	}

	/**
	 * Returns the inverse of the matrix
	 */
	Matrix3 Inverse() const
	{
		Matrix3 kInverse = *this;
		kt_bool haveInverse = InverseFast(kInverse, 1e-14);
		if (haveInverse == false)
		{
			assert(false);
		}
		return kInverse;
	}

	/**
	 * Internal helper method for inverse matrix calculation
	 * This code is lifted from the OgreMatrix3 class!!
	 */
	kt_bool InverseFast(Matrix3& rkInverse, kt_double fTolerance = KT_TOLERANCE) const
	{
		// Invert a 3x3 using cofactors.  This is about 8 times faster than
		// the Numerical Recipes code which uses Gaussian elimination.
		rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
		rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
		rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
		rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
		rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
		rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
		rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
		rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
		rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];

		kt_double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] +
						m_Matrix[0][1]*rkInverse.m_Matrix[1][0] +
						m_Matrix[0][2]*rkInverse.m_Matrix[2][0];

		if (fabs(fDet) <= fTolerance)
		{
			return false;
		}

		kt_double fInvDet = 1.0/fDet;
		for (size_t row = 0; row < 3; row++)
		{
			for (size_t col = 0; col < 3; col++)
			{
				rkInverse.m_Matrix[row][col] *= fInvDet;
			}
		}

		return true;
	}

public:
	/**
	 * Assignment operator
	 */
	inline Matrix3& operator = (const Matrix3& rOther)
	{
		memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
		return *this;
	}

	/**
	 * Matrix element access, allows use of construct mat(r, c)
	 * @param row
	 * @param column
	 * @return reference to mat(r,c)
	 */
	inline kt_double& operator()(kt_int32u row, kt_int32u column)
	{
		return m_Matrix[row][column];
	}

	/**
	 * Read-only matrix element access, allows use of construct mat(r, c)
	 * @param row
	 * @param column
	 * @return mat(r,c)
	 */
	inline kt_double operator()(kt_int32u row, kt_int32u column) const
	{
		return m_Matrix[row][column];
	}

	/**
	 * Binary Matrix3 multiplication.
	 * @param rOther
	 * @return Matrix3 product
	 */
	Matrix3 operator * (const Matrix3& rOther) const
	{
		Matrix3 product;

		for (size_t row = 0; row < 3; row++)
		{
			for (size_t col = 0; col < 3; col++)
			{
				product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] +
											m_Matrix[row][1]*rOther.m_Matrix[1][col] +
											m_Matrix[row][2]*rOther.m_Matrix[2][col];
			}
		}

		return product;
	}

	/**
	 * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
	 * @param rPose2
	 * @return Pose2 product
	 */
	inline Pose2 operator * (const Pose2& rPose2) const
	{
		Pose2 pose2;

		pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
					rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
		pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
					rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
		pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
						rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

		return pose2;
	}

	/**
	 * In place Matrix3 add.
	 * @param rkMatrix
	 */
	inline void operator += (const Matrix3& rkMatrix)
	{
		for (kt_int32u row = 0; row < 3; row++)
		{
			for (kt_int32u col = 0; col < 3; col++)
			{
				m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
			}
		}
	}

private:
	kt_double m_Matrix[3][3];
};  // Matrix3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class BoundingBox2
{
public:
	/*
	 * Default constructor
	 */
	BoundingBox2()
	: m_Minimum(999999999999999999.99999, 999999999999999999.99999)
	, m_Maximum(-999999999999999999.99999, -999999999999999999.99999)
	{
	}

public:
	/**
	 * Get bounding box minimum
	 */
	inline const Vector2<kt_double>& GetMinimum() const
	{
		return m_Minimum;
	}

	/**
	 * Set bounding box minimum
	 */
	inline void SetMinimum(const Vector2<kt_double>& mMinimum)
	{
		m_Minimum = mMinimum;
	}

	/**
	 * Get bounding box maximum
	 */
	inline const Vector2<kt_double>& GetMaximum() const
	{
		return m_Maximum;
	}

	/**
	 * Set bounding box maximum
	 */
	inline void SetMaximum(const Vector2<kt_double>& rMaximum)
	{
		m_Maximum = rMaximum;
	}

	/**
	 * Get the size of the bounding box
	 */
	inline Size2<kt_double> GetSize() const
	{
		Vector2<kt_double> size = m_Maximum - m_Minimum;

		return Size2<kt_double>(size.GetX(), size.GetY());
	}

	/**
	 * Add vector to bounding box
	 */
	inline void Add(const Vector2<kt_double>& rPoint)
	{
		m_Minimum.MakeFloor(rPoint);
		m_Maximum.MakeCeil(rPoint);
	}

	/**
	 * Add other bounding box to bounding box
	 */
	inline void Add(const BoundingBox2& rBoundingBox)
	{
		Add(rBoundingBox.GetMinimum());
		Add(rBoundingBox.GetMaximum());
	}

	/**
	 * Whether the given point is in the bounds of this box
	 * @param rPoint
	 * @return in bounds?
	 */
	inline kt_bool IsInBounds(const Vector2<kt_double>& rPoint) const
	{
		return (math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
				math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
	}

private:
	Vector2<kt_double> m_Minimum;
	Vector2<kt_double> m_Maximum;
};  // BoundingBox2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Implementation of a Pose2 transform
 */
class Transform
{
public:
	/**
	 * Constructs a transformation from the origin to the given pose
	 * @param rPose pose
	 */
	Transform(const Pose2& rPose)
	{
		SetTransform(Pose2(), rPose);
	}

	/**
	 * Constructs a transformation from the first pose to the second pose
	 * @param rPose1 first pose
	 * @param rPose2 second pose
	 */
	Transform(const Pose2& rPose1, const Pose2& rPose2)
	{
		SetTransform(rPose1, rPose2);
	}

public:
	/**
	 * Transforms the pose according to this transform
	 * @param rSourcePose pose to transform from
	 * @return transformed pose
	 */
	inline Pose2 TransformPose(const Pose2& rSourcePose)
	{
		Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
		kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

		return Pose2(newPosition.GetPosition(), angle);
	}

	/**
	 * Inverse transformation of the pose according to this transform
	 * @param rSourcePose pose to transform from
	 * @return transformed pose
	 */
	inline Pose2 InverseTransformPose(const Pose2& rSourcePose)
	{
		Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
		kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

		// components of transform
		return Pose2(newPosition.GetPosition(), angle);
	}

private:
	/**
	 * Sets this to be the transformation from the first pose to the second pose
	 * @param rPose1 first pose
	 * @param rPose2 second pose
	 */
	void SetTransform(const Pose2& rPose1, const Pose2& rPose2)
	{
		if (rPose1 == rPose2)
		{
			m_Rotation.SetToIdentity();
			m_InverseRotation.SetToIdentity();
			m_Transform = Pose2();
			return;
		}

		// heading transformation
		m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
		m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

		// position transformation
		Pose2 newPosition;
		if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
		{
			newPosition = rPose2 - m_Rotation * rPose1;
		}
		else
		{
			newPosition = rPose2;
		}

		m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
	}

private:
	// pose transformation
	Pose2 m_Transform;

	Matrix3 m_Rotation;
	Matrix3 m_InverseRotation;
};  // Transform
	
}  // namespace KartoScanMatcher

#endif  // _DATA_STRUCTURE_H_