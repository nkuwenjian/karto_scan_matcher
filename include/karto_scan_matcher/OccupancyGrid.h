/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KARTO_SCAN_MATCHER_OCCUPANCY_GRID_H
#define KARTO_SCAN_MATCHER_OCCUPANCY_GRID_H

#include "karto_scan_matcher/DataStructure.h"
#include "karto_scan_matcher/Grid.h"

#include <string.h>

namespace KartoScanMatcher
{
/**
 * Occupancy grid definition. See GridStates for possible grid values.
 */
class OccupancyGrid : public Grid<kt_int8u>
{
public:
  /**
   * Constructs an occupancy grid of given size
   * @param width
   * @param height
   * @param rOffset
   * @param resolution
   */
  OccupancyGrid(kt_int32s width, kt_int32s height, const Vector2<kt_double>& rOffset, kt_double resolution)
    : Grid<kt_int8u>(width, height)
    , m_pCellPassCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
    , m_pCellHitsCnt(Grid<kt_int32u>::CreateGrid(0, 0, resolution))
  {
    if (math::DoubleEqual(resolution, 0.0))
    {
      throw("Resolution cannot be 0");
    }

    m_pMinPassThrough = 2;
    m_pOccupancyThreshold = 0.1;

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  /**
   * Destructor
   */
  virtual ~OccupancyGrid()
  {
    delete m_pCellPassCnt;
    delete m_pCellHitsCnt;
  }

public:
  /**
   * Create an occupancy grid from the given scans using the given resolution
   * @param rScans
   * @param resolution
   */
  static OccupancyGrid* CreateFromScans(const LocalizedRangeScanVector& rScans, kt_double resolution)
  {
    if (rScans.empty())
    {
      return NULL;
    }

    kt_int32s width, height;
    Vector2<kt_double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;
  }

  /**
   * Make a clone
   * @return occupancy grid clone
   */
  OccupancyGrid* Clone() const
  {
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(GetWidth(), GetHeight(), GetCoordinateConverter()->GetOffset(),
                                                      1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt = m_pCellPassCnt->Clone();
    pOccupancyGrid->m_pCellHitsCnt = m_pCellHitsCnt->Clone();

    return pOccupancyGrid;
  }

  /**
   * Check if grid point is free
   * @param rPose
   * @return whether the cell at the given point is free space
   */
  virtual kt_bool IsFree(const Vector2<kt_int32s>& rPose) const
  {
    kt_int8u* pOffsets = reinterpret_cast<kt_int8u*>(GetDataPointer(rPose));
    if (*pOffsets == GridStates_Free)
    {
      return true;
    }

    return false;
  }

  /**
   * Casts a ray from the given point (up to the given max range)
   * and returns the distance to the closest obstacle
   * @param rPose2
   * @param maxRange
   * @return distance to closest obstacle
   */
  virtual kt_double RayCast(const Pose2& rPose2, kt_double maxRange) const
  {
    double scale = GetCoordinateConverter()->GetScale();

    kt_double x = rPose2.GetX();
    kt_double y = rPose2.GetY();
    kt_double theta = rPose2.GetHeading();

    kt_double sinTheta = sin(theta);
    kt_double cosTheta = cos(theta);

    kt_double xStop = x + maxRange * cosTheta;
    kt_double xSteps = 1 + fabs(xStop - x) * scale;

    kt_double yStop = y + maxRange * sinTheta;
    kt_double ySteps = 1 + fabs(yStop - y) * scale;

    kt_double steps = math::Maximum(xSteps, ySteps);
    kt_double delta = maxRange / steps;
    kt_double distance = delta;

    for (kt_int32u i = 1; i < steps; i++)
    {
      kt_double x1 = x + distance * cosTheta;
      kt_double y1 = y + distance * sinTheta;

      Vector2<kt_int32s> gridIndex = WorldToGrid(Vector2<kt_double>(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex))
      {
        distance = (i + 1) * delta;
      }
      else
      {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  /**
   * Sets the minimum number of beams that must pass through a cell before it
   * will be considered to be occupied or unoccupied.
   * This prevents stray beams from messing up the map.
   */
  void SetMinPassThrough(kt_int32u count)
  {
    m_pMinPassThrough = count;
  }

  /**
   * Sets the minimum ratio of beams hitting cell to beams passing through
   * cell for cell to be marked as occupied.
   */
  void SetOccupancyThreshold(kt_double thresh)
  {
    m_pOccupancyThreshold = thresh;
  }

protected:
  /**
   * Get cell hit grid
   * @return Grid<kt_int32u>*
   */
  virtual Grid<kt_int32u>* GetCellHitsCounts()
  {
    return m_pCellHitsCnt;
  }

  /**
   * Get cell pass grid
   * @return Grid<kt_int32u>*
   */
  virtual Grid<kt_int32u>* GetCellPassCounts()
  {
    return m_pCellPassCnt;
  }

protected:
  /**
   * Calculate grid dimensions from localized range scans
   * @param rScans
   * @param resolution
   * @param rWidth
   * @param rHeight
   * @param rOffset
   */
  static void ComputeDimensions(const LocalizedRangeScanVector& rScans, kt_double resolution, kt_int32s& rWidth,
                                kt_int32s& rHeight, Vector2<kt_double>& rOffset)
  {
    BoundingBox2 boundingBox;
    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      boundingBox.Add((*iter)->GetBoundingBox());
    }

    kt_double scale = 1.0 / resolution;
    Size2<kt_double> size = boundingBox.GetSize();

    rWidth = static_cast<kt_int32s>(math::Round(size.GetWidth() * scale));
    rHeight = static_cast<kt_int32s>(math::Round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  /**
   * Create grid using scans
   * @param rScans
   */
  virtual void CreateFromScans(const LocalizedRangeScanVector& rScans)
  {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    const_forEach(LocalizedRangeScanVector, &rScans)
    {
      LocalizedRangeScan* pScan = *iter;
      AddScan(pScan);
    }

    Update();
  }

  /**
   * Adds the scan's information to this grid's counters (optionally
   * update the grid's cells' occupancy status)
   * @param pScan
   * @param doUpdate whether to update the grid's cell's occupancy status
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual kt_bool AddScan(LocalizedRangeScan* pScan, kt_bool doUpdate = false)
  {
    kt_double rangeThreshold = pScan->GetLaserRangeFinder()->GetRangeThreshold();
    kt_double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();
    kt_double minRange = pScan->GetLaserRangeFinder()->GetMinimumRange();

    Vector2<kt_double> scanPosition = pScan->GetSensorPose().GetPosition();

    // get scan point readings
    const PointVectorDouble& rPointReadings = pScan->GetPointReadings(false);

    kt_bool isAllInMap = true;

    // draw lines from scan position to all point readings
    int pointIndex = 0;
    const_forEachAs(PointVectorDouble, &rPointReadings, pointsIter)
    {
      Vector2<kt_double> point = *pointsIter;
      kt_double rangeReading = pScan->GetRangeReadings()[pointIndex];
      kt_bool isEndPointValid = rangeReading < (rangeThreshold - KT_TOLERANCE);

      if (rangeReading <= minRange || rangeReading >= maxRange || std::isnan(rangeReading))
      {
        // ignore these readings
        pointIndex++;
        continue;
      }
      else if (rangeReading >= rangeThreshold)
      {
        // trace up to range reading
        kt_double ratio = rangeThreshold / rangeReading;
        kt_double dx = point.GetX() - scanPosition.GetX();
        kt_double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      kt_bool isInMap = RayTrace(scanPosition, point, isEndPointValid, doUpdate);
      if (!isInMap)
      {
        isAllInMap = false;
      }

      pointIndex++;
    }

    return isAllInMap;
  }

  /**
   * Traces a beam from the start position to the end position marking
   * the bookkeeping arrays accordingly.
   * @param rWorldFrom start position of beam
   * @param rWorldTo end position of beam
   * @param isEndPointValid is the reading within the range threshold?
   * @param doUpdate whether to update the cells' occupancy status immediately
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual kt_bool RayTrace(const Vector2<kt_double>& rWorldFrom, const Vector2<kt_double>& rWorldTo,
                           kt_bool isEndPointValid, kt_bool doUpdate = false)
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    Vector2<kt_int32s> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2<kt_int32s> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    // CellUpdater* pCellUpdater = doUpdate ? m_pCellUpdater : NULL;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(), gridTo.GetY());

    // for the end point
    if (isEndPointValid)
    {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo))
      {
        kt_int32s index = m_pCellPassCnt->GridIndex(gridTo, false);

        kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;
      }
    }

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  /**
   * Updates a single cell's value based on the given counters
   * @param pCell
   * @param cellPassCnt
   * @param cellHitCnt
   */
  virtual void UpdateCell(kt_int8u* pCell, kt_int32u cellPassCnt, kt_int32u cellHitCnt)
  {
    if (cellPassCnt > m_pMinPassThrough)
    {
      kt_double hitRatio = static_cast<kt_double>(cellHitCnt) / static_cast<kt_double>(cellPassCnt);

      if (hitRatio > m_pOccupancyThreshold)
      {
        *pCell = GridStates_Occupied;
      }
      else
      {
        *pCell = GridStates_Free;
      }
    }
  }

  /**
   * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
   */
  virtual void Update()
  {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    // clear grid
    Clear();

    // set occupancy status of cells
    kt_int8u* pDataPtr = GetDataPointer();
    kt_int32u* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    kt_int32u* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    kt_int32u nBytes = GetDataSize();
    for (kt_int32u i = 0; i < nBytes; i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++)
    {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(kt_int32s width, kt_int32s height)
  {
    Grid<kt_int8u>::Resize(width, height);
    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

protected:
  /**
   * Counters of number of times a beam passed through a cell
   */
  Grid<kt_int32u>* m_pCellPassCnt;

  /**
   * Counters of number of times a beam ended at a cell
   */
  Grid<kt_int32u>* m_pCellHitsCnt;

private:
  /**
   * Restrict the copy constructor
   */
  OccupancyGrid(const OccupancyGrid&);

  /**
   * Restrict the assignment operator
   */
  const OccupancyGrid& operator=(const OccupancyGrid&);

private:
  ////////////////////////////////////////////////////////////
  // NOTE: These two values are dependent on the resolution.  If the resolution is too small,
  // then not many beams will hit the cell!

  // Number of beams that must pass through a cell before it will be considered to be occupied
  // or unoccupied.  This prevents stray beams from messing up the map.
  kt_int32u m_pMinPassThrough;

  // Minimum ratio of beams hitting cell to beams passing through cell for cell to be marked as occupied
  kt_double m_pOccupancyThreshold;
};  // OccupancyGrid

}  // namespace KartoScanMatcher

#endif  // KARTO_SCAN_MATCHER_OCCUPANCY_GRID_H
