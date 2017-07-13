#ifndef _COORDINATE_CONVERTER_H_
#define _COORDINATE_CONVERTER_H_

#include <karto_scan_matcher/DataStructure.h>

namespace KartoScanMatcher
{
/**
 * The CoordinateConverter class is used to convert coordinates between world and grid coordinates
 * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
 * Default scale for coordinate converter is 20 that converters to 1 pixel = 0.05 meter
 */
class CoordinateConverter
{
public:
	/**
	 * Default constructor
	 */
	CoordinateConverter()
	: m_Scale(20.0)
	{
	}

public:
	/**
	 * Scales the value
	 * @param value
	 * @return scaled value
	 */
	inline kt_double Transform(kt_double value)
	{
		return value * m_Scale;
	}

	/**
	 * Converts the point from world coordinates to grid coordinates
	 * @param rWorld world coordinate
	 * @param flipY
	 * @return grid coordinate
	 */
	inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
	{
		kt_double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
		kt_double gridY = 0.0;

		if (flipY == false)
		{
			gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
		}
		else
		{
			gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) * m_Scale;
		}

		return Vector2<kt_int32s>(static_cast<kt_int32s>(math::Round(gridX)), static_cast<kt_int32s>(math::Round(gridY)));
	}

	/**
	 * Converts the point from grid coordinates to world coordinates
	 * @param rGrid world coordinate
	 * @param flipY
	 * @return world coordinate
	 */
	inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
	{
		kt_double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
		kt_double worldY = 0.0;

		if (flipY == false)
		{
			worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
		}
		else
		{
			worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
		}

		return Vector2<kt_double>(worldX, worldY);
	}

	/**
	 * Gets the scale
	 * @return scale
	 */
	inline kt_double GetScale() const
	{
		return m_Scale;
	}

	/**
	 * Sets the scale
	 * @param scale
	 */
	inline void SetScale(kt_double scale)
	{
		m_Scale = scale;
	}

	/**
	 * Gets the offset
	 * @return offset
	 */
	inline const Vector2<kt_double>& GetOffset() const
	{
		return m_Offset;
	}

	/**
	 * Sets the offset
	 * @param rOffset
	 */
	inline void SetOffset(const Vector2<kt_double>& rOffset)
	{
		m_Offset = rOffset;
	}

	/**
	 * Sets the size
	 * @param rSize
	 */
	inline void SetSize(const Size2<kt_int32s>& rSize)
	{
		m_Size = rSize;
	}

	/**
	 * Gets the size
	 * @return size
	 */
	inline const Size2<kt_int32s>& GetSize() const
	{
		return m_Size;
	}

	/**
	 * Gets the resolution
	 * @return resolution
	 */
	inline kt_double GetResolution() const
	{
		return 1.0 / m_Scale;
	}

	/**
	 * Sets the resolution
	 * @param resolution
	 */
	inline void SetResolution(kt_double resolution)
	{
		m_Scale = 1.0 / resolution;
	}

	/**
	 * Gets the bounding box
	 * @return bounding box
	 */
	inline BoundingBox2 GetBoundingBox() const
	{
		BoundingBox2 box;

		kt_double minX = GetOffset().GetX();
		kt_double minY = GetOffset().GetY();
		kt_double maxX = minX + GetSize().GetWidth() * GetResolution();
		kt_double maxY = minY + GetSize().GetHeight() * GetResolution();

		box.SetMinimum(GetOffset());
		box.SetMaximum(Vector2<kt_double>(maxX, maxY));
		return box;
	}

private:
	Size2<kt_int32s> m_Size;
	kt_double m_Scale;

	Vector2<kt_double> m_Offset;
};  // CoordinateConverter

}  // namespace KartoScanMatcher

#endif  // _COORDINATE_CONVERTER_H_