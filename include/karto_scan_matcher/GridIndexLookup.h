#ifndef KARTO_SCAN_MATCHER_GRID_INDEX_LOOKUP_H_
#define KARTO_SCAN_MATCHER_GRID_INDEX_LOOKUP_H_

#include <karto_scan_matcher/DataStructure.h>
#include <karto_scan_matcher/Grid.h>
#include <karto_scan_matcher/LaserSensor.h>

#include <assert.h>
#include <math.h>
#include <string.h>

namespace KartoScanMatcher
{
/**
 * An array that can be resized as long as the size
 * does not exceed the initial capacity
 */
class LookupArray
{
public:
  /**
   * Constructs lookup array
   */
  LookupArray()
  : m_pArray(NULL)
  , m_Capacity(0)
  , m_Size(0)
  {
  }

  /**
   * Destructor
   */
  virtual ~LookupArray()
  {
    assert(m_pArray != NULL);

    delete[] m_pArray;
    m_pArray = NULL;
  }

public:
  /**
   * Clear array
   */
  void Clear()
  {
    memset(m_pArray, 0, sizeof(kt_int32s) * m_Capacity);
  }

  /**
   * Gets size of array
   * @return array size
   */
  kt_int32u GetSize() const
  {
    return m_Size;
  }

  /**
   * Sets size of array (resize if not big enough)
   * @param size
   */
  void SetSize(kt_int32u size)
  {
    assert(size != 0);

    if (size > m_Capacity)
    {
      if (m_pArray != NULL)
      {
        delete [] m_pArray;
      }
      m_Capacity = size;
      m_pArray = new kt_int32s[m_Capacity];
    }

    m_Size = size;
  }

  /**
   * Gets reference to value at given index
   * @param index
   * @return reference to value at index
   */
  inline kt_int32s& operator [] (kt_int32u index)
  {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets value at given index
   * @param index
   * @return value at index
   */
  inline kt_int32s operator [] (kt_int32u index) const
  {
    assert(index < m_Size);

    return m_pArray[index];
  }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline kt_int32s* GetArrayPointer()
  {
    return m_pArray;
  }

  /**
   * Gets array pointer
   * @return array pointer
   */
  inline kt_int32s* GetArrayPointer() const
  {
    return m_pArray;
  }

private:
  kt_int32s* m_pArray;
  kt_int32u m_Capacity;
  kt_int32u m_Size;
};  // LookupArray

/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
 * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
 * calculated.  So when calculating the particle probability at a specific angle, the index table is used
 * to look up probability in probability grid!
 *
 */
template<typename T>
class GridIndexLookup
{
public:
  /**
   * Construct a GridIndexLookup with a grid
   * @param pGrid
   */
  GridIndexLookup(Grid<T>* pGrid)
  : m_pGrid(pGrid)
  , m_Capacity(0)
  , m_Size(0)
  , m_ppLookupArray(NULL)
  {
  }

  /**
   * Destructor
   */
  virtual ~GridIndexLookup()
  {
    DestroyArrays();
  }

public:
  /**
   * Gets the lookup array for a particular angle index
   * @param index
   * @return lookup array
   */
  const LookupArray* GetLookupArray(kt_int32u index) const
  {
    assert(math::IsUpTo(index, m_Size));

    return m_ppLookupArray[index];
  }

  /**
   * Get angles
   * @return std::vector<kt_double>& angles
   */
  const std::vector<kt_double>& GetAngles() const
  {
    return m_Angles;
  }

  /**
   * Compute lookup table of the points of the given scan for the given angular space
   * @param pScan the scan
   * @param angleCenter
   * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
   * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
   */
  void ComputeOffsets(LocalizedRangeScan* pScan,
            kt_double angleCenter,
            kt_double angleOffset,
            kt_double angleResolution)
  {
    assert(angleOffset != 0.0);
    assert(angleResolution != 0.0);

    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
    SetSize(nAngles);

    //////////////////////////////////////////////////////
    // convert points into local coordinates of scan pose

    const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

    // compute transform to scan pose
    Transform transform(pScan->GetSensorPose());

    Pose2Vector localPoints;
    const_forEach(PointVectorDouble, &rPointReadings)
    {
      // do inverse transform to get points in local coordinates
      Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
      localPoints.push_back(vec);
    }

    //////////////////////////////////////////////////////
    // create lookup array for different angles
    kt_double angle = 0.0;
    kt_double startAngle = angleCenter - angleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * angleResolution;
      ComputeOffsets(angleIndex, angle, localPoints, pScan);
    }
    // assert(math::DoubleEqual(angle, angleCenter + angleOffset));
  }

private:
  /**
   * Compute lookup value of points for given angle
   * @param angleIndex
   * @param angle
   * @param rLocalPoints
   */
  void ComputeOffsets(kt_int32u angleIndex, kt_double angle, const Pose2Vector& rLocalPoints, LocalizedRangeScan* pScan)
  {
    m_ppLookupArray[angleIndex]->SetSize(static_cast<kt_int32u>(rLocalPoints.size()));
    m_Angles.at(angleIndex) = angle;

    // set up point array by computing relative offsets to points readings
    // when rotated by given angle

    const Vector2<kt_double>& rGridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();

    kt_double cosine = cos(angle);
    kt_double sine = sin(angle);

    kt_int32u readingIndex = 0;

    kt_int32s* pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

    kt_double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();

    const_forEach(Pose2Vector, &rLocalPoints)
    {
      const Vector2<kt_double>& rPosition = iter->GetPosition();

      if (isnan(pScan->GetRangeReadings()[readingIndex]) || isinf(pScan->GetRangeReadings()[readingIndex]))
      {
        pAngleIndexPointer[readingIndex] = INVALID_SCAN;
        readingIndex++;
        continue;
      }


      // counterclockwise rotation and that rotation is about the origin (0, 0).
      Vector2<kt_double> offset;
      offset.SetX(cosine * rPosition.GetX() - sine * rPosition.GetY());
      offset.SetY(sine * rPosition.GetX() + cosine * rPosition.GetY());

      // have to compensate for the grid offset when getting the grid index
      Vector2<kt_int32s> gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);

      // use base GridIndex to ignore ROI
      kt_int32s lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

      pAngleIndexPointer[readingIndex] = lookupIndex;

      readingIndex++;
    }
    assert(readingIndex == rLocalPoints.size());
  }

  /**
   * Sets size of lookup table (resize if not big enough)
   * @param size
   */
  void SetSize(kt_int32u size)
  {
    assert(size != 0);

    if (size > m_Capacity)
    {
      if (m_ppLookupArray != NULL)
      {
        DestroyArrays();
      }

      m_Capacity = size;
      m_ppLookupArray = new LookupArray*[m_Capacity];
      for (kt_int32u i = 0; i < m_Capacity; i++)
      {
        m_ppLookupArray[i] = new LookupArray();
      }
    }

    m_Size = size;

    m_Angles.resize(size);
  }

  /**
   * Delete the arrays
   */
  void DestroyArrays()
  {
    for (kt_int32u i = 0; i < m_Capacity; i++)
    {
      delete m_ppLookupArray[i];
    }

    delete[] m_ppLookupArray;
    m_ppLookupArray = NULL;
  }

private:
  Grid<T>* m_pGrid;

  kt_int32u m_Capacity;
  kt_int32u m_Size;

  LookupArray **m_ppLookupArray;

  // for sanity check
  std::vector<kt_double> m_Angles;
};  // GridIndexLookup
  
}  // namespace KartoScanMatcher

#endif  // KARTO_SCAN_MATCHER_GRID_INDEX_LOOKUP_H_