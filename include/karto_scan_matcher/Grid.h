#ifndef KARTO_SCAN_MATCHER_GRID_H_
#define KARTO_SCAN_MATCHER_GRID_H_

#include <karto_scan_matcher/DataStructure.h>
#include <karto_scan_matcher/CoordinateConverter.h>

#include <cstdlib>
#include <sstream>

namespace KartoScanMatcher
{
/**
 * Defines a grid class
 */
template<typename T>
class Grid
{
public:
  /**
   * Creates a grid of given size and resolution
   * @param width
   * @param height
   * @param resolution
   * @return grid pointer
   */
  static Grid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution)
  {
    Grid* pGrid = new Grid(width, height);

    pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);

    return pGrid;
  }

  /**
   * Destructor
   */
  virtual ~Grid()
  {
    delete [] m_pData;
    delete m_pCoordinateConverter;
  }

public:
  /**
   * Clear out the grid data
   */
  void Clear()
  {
    memset(m_pData, 0, GetDataSize() * sizeof(T));
  }

  /**
   * Returns a clone of this grid
   * @return grid clone
   */
  Grid* Clone()
  {
    Grid* pGrid = CreateGrid(GetWidth(), GetHeight(), GetResolution());
    pGrid->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    memcpy(pGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    return pGrid;
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(kt_int32s width, kt_int32s height)
  {
    m_Width = width;                                      // m_Width = 31
    m_Height = height;
    m_WidthStep = math::AlignValue<kt_int32s>(width, 8);  // m_WidthStep = 32

    if (m_pData != NULL)
    {
      delete[] m_pData;
      m_pData = NULL;
    }

    try
    {
      m_pData = new T[GetDataSize()];

      if (m_pCoordinateConverter == NULL)
      {
        m_pCoordinateConverter = new CoordinateConverter();
      }

      m_pCoordinateConverter->SetSize(Size2<kt_int32s>(width, height));
    }
    catch(...)
    {
      m_pData = NULL;

      m_Width = 0;
      m_Height = 0;
      m_WidthStep = 0;
    }

    Clear();
  }

  /**
   * Checks whether the given coordinates are valid grid indices
   * @param rGrid
   */
  inline kt_bool IsValidGridIndex(const Vector2<kt_int32s>& rGrid) const
  {
    return (math::IsUpTo(rGrid.GetX(), m_Width) && math::IsUpTo(rGrid.GetY(), m_Height));
  }

  /**
   * Gets the index into the data pointer of the given grid coordinate
   * @param rGrid
   * @param boundaryCheck default value is true
   * @return grid index
   */
  virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
  {
    if (boundaryCheck == true)
    {
      if (IsValidGridIndex(rGrid) == false)
      {
        std::stringstream error;
        error << "Index out of range.  Index must be between [0; "
            << m_Width << ") and [0; " << m_Height << ")";
        throw (error.str());
      }
    }

    kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);

    if (boundaryCheck == true)
    {
      assert(math::IsUpTo(index, GetDataSize()));
    }

    return index;
  }

  /**
   * Gets the grid coordinate from an index
   * @param index
   * @return grid coordinate
   */
  Vector2<kt_int32s> IndexToGrid(kt_int32s index) const
  {
    Vector2<kt_int32s> grid;

    grid.SetY(index / m_WidthStep);
    grid.SetX(index - grid.GetY() * m_WidthStep);

    return grid;
  }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
  {
    return GetCoordinateConverter()->WorldToGrid(rWorld, flipY);
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
  {
    return GetCoordinateConverter()->GridToWorld(rGrid, flipY);
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T* GetDataPointer(const Vector2<kt_int32s>& rGrid)
  {
    kt_int32s index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T* GetDataPointer(const Vector2<kt_int32s>& rGrid) const
  {
    kt_int32s index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets the width of the grid
   * @return width of the grid
   */
  inline kt_int32s GetWidth() const
  {
    return m_Width;
  }

  /**
   * Gets the height of the grid
   * @return height of the grid
   */
  inline kt_int32s GetHeight() const
  {
    return m_Height;
  }

  /**
   * Get the size as a Size2<kt_int32s>
   * @return size of the grid
   */
  inline const Size2<kt_int32s> GetSize() const
  {
    return Size2<kt_int32s>(m_Width, m_Height);
  }

  /**
   * Gets the width step in bytes
   * @return width step
   */
  inline kt_int32s GetWidthStep() const
  {
    return m_WidthStep;
  }

  /**
   * Gets the grid data pointer
   * @return data pointer
   */
  inline T* GetDataPointer()
  {
    return m_pData;
  }

  /**
   * Gets const grid data pointer
   * @return data pointer
   */
  inline T* GetDataPointer() const
  {
    return m_pData;
  }

  /**
   * Gets the allocated grid size in bytes
   * @return data size
   */
  inline kt_int32s GetDataSize() const
  {
    return m_WidthStep * m_Height;
  }

  /**
   * Get value at given grid coordinate
   * @param rGrid grid coordinate
   * @return value
   */
  inline T GetValue(const Vector2<kt_int32s>& rGrid) const
  {
    kt_int32s index = GridIndex(rGrid);
    return m_pData[index];
  }

  /**
   * Gets the coordinate converter for this grid
   * @return coordinate converter
   */
  inline CoordinateConverter* GetCoordinateConverter() const
  {
    return m_pCoordinateConverter;
  }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline kt_double GetResolution() const
  {
    return GetCoordinateConverter()->GetResolution();
  }

  /**
   * Gets the grids bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const
  {
    return GetCoordinateConverter()->GetBoundingBox();
  }

  /**
   * Increments all the grid cells from (x0, y0) to (x1, y1);
   * if applicable, apply f to each cell traced
   * @param x0
   * @param y0
   * @param x1
   * @param y1
   * @param f
   */
  void TraceLine(kt_int32s x0, kt_int32s y0, kt_int32s x1, kt_int32s y1)
  {
    kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1)
    {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }

    kt_int32s deltaX = x1 - x0;
    kt_int32s deltaY = abs(y1 - y0);
    kt_int32s error = 0;
    kt_int32s ystep;
    kt_int32s y = y0;

    if (y0 < y1)
    {
      ystep = 1;
    }
    else
    {
      ystep = -1;
    }

    kt_int32s pointX;
    kt_int32s pointY;
    for (kt_int32s x = x0; x <= x1; x++)
    {
      if (steep)
      {
        pointX = y;
        pointY = x;
      }
      else
      {
        pointX = x;
        pointY = y;
      }

      error += deltaY;

      if (2 * error >= deltaX)
      {
        y += ystep;
        error -= deltaX;
      }

      Vector2<kt_int32s> gridIndex(pointX, pointY);
      if (IsValidGridIndex(gridIndex))
      {
        kt_int32s index = GridIndex(gridIndex, false);
        T* pGridPointer = GetDataPointer();
        pGridPointer[index]++;
      }
    }
  }

protected:
  /**
   * Constructs grid of given size
   * @param width
   * @param height
   */
  Grid(kt_int32s width, kt_int32s height)
  : m_pData(NULL)
  , m_pCoordinateConverter(NULL)
  {
    Resize(width, height);
  }

private:
  kt_int32s m_Width;       // width of grid
  kt_int32s m_Height;      // height of grid
  kt_int32s m_WidthStep;   // 8 bit aligned width of grid
  T* m_pData;              // grid data

  // coordinate converter to convert between world coordinates and grid coordinates
  CoordinateConverter* m_pCoordinateConverter;
};  // Grid
  
}  //  namespace KartoScanMatcher

#endif  // KARTO_SCAN_MATCHER_GRID_H_