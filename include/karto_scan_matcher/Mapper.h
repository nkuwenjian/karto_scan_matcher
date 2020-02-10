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

#ifndef KARTO_SCAN_MATCHER_MAPPER_H
#define KARTO_SCAN_MATCHER_MAPPER_H

#include <map>
#include <vector>

#include "karto_scan_matcher/DataStructure.h"
#include "karto_scan_matcher/CorrelationGrid.h"
#include "karto_scan_matcher/Grid.h"
#include "karto_scan_matcher/LaserSensor.h"
#include "karto_scan_matcher/GridIndexLookup.h"
#include "karto_scan_matcher/MapperSensorManager.h"

namespace KartoScanMatcher
{
class Mapper;
/**
 * Scan matcher
 */
class KARTO_EXPORT ScanMatcher
{
public:
  /**
   * Destructor
   */
  virtual ~ScanMatcher();

public:
  /**
   * Create a scan matcher with the given parameters
   */
  static ScanMatcher* Create(Mapper* pMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation,
                             kt_double rangeThreshold);

  /**
   * Match given scan against set of scans
   * @param pScan scan being scan-matched
   * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
   * @param rMean output parameter of mean (best pose) of match
   * @param rCovariance output parameter of covariance of match
   * @param doPenalize whether to penalize matches further from the search center
   * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
   * @return strength of response
   */
  kt_double MatchScan(LocalizedRangeScan* pScan, const LocalizedRangeScanVector& rBaseScans, Pose2& rMean,
                      Matrix3& rCovariance, kt_bool doPenalize = true, kt_bool doRefineMatch = true);

  /**
   * Finds the best pose for the scan centering the search in the correlation grid
   * at the given pose and search in the space by the vector and angular offsets
   * in increments of the given resolutions
   * @param pScan scan to match against correlation grid
   * @param rSearchCenter the center of the search space
   * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
   * @param rSearchSpaceResolution how fine a granularity to search in the search space
   * @param searchAngleOffset searches poses in the angles offset by this angle around search center
   * @param searchAngleResolution how fine a granularity to search in the angular search space
   * @param doPenalize whether to penalize matches further from the search center
   * @param rMean output parameter of mean (best pose) of match
   * @param rCovariance output parameter of covariance of match
   * @param doingFineMatch whether to do a finer search after coarse search
   * @return strength of response
   */
  kt_double CorrelateScan(LocalizedRangeScan* pScan, const Pose2& rSearchCenter,
                          const Vector2<kt_double>& rSearchSpaceOffset,
                          const Vector2<kt_double>& rSearchSpaceResolution, kt_double searchAngleOffset,
                          kt_double searchAngleResolution, kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance,
                          kt_bool doingFineMatch);

  /**
   * Gets the correlation grid data (for debugging)
   * @return correlation grid
   */
  inline CorrelationGrid* GetCorrelationGrid() const
  {
    return m_pCorrelationGrid;
  }

private:
  /**
   * Marks cells where scans' points hit as being occupied
   * @param rScans scans whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   */
  void AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint);

  /**
   * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
   * @param pScan scan whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   * @param doSmear whether the points will be smeared
   */
  void AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear = true);

  /**
   * Compute which points in a scan are on the same side as the given viewpoint
   * @param pScan
   * @param rViewPoint
   * @return points on the same side
   */
  PointVectorDouble FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const;

  /**
   * Get response at given position for given rotation (only look up valid points)
   * @param angleIndex
   * @param gridPositionIndex
   * @return response
   */
  kt_double GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const;

protected:
  /**
   * Default constructor
   */
  ScanMatcher(Mapper* pMapper)
    : m_pMapper(pMapper), m_pCorrelationGrid(NULL), m_pSearchSpaceProbs(NULL), m_pGridLookup(NULL)
  {
  }

private:
  Mapper* m_pMapper;

  CorrelationGrid* m_pCorrelationGrid;
  Grid<kt_double>* m_pSearchSpaceProbs;

  GridIndexLookup<kt_int8u>* m_pGridLookup;
};  // ScanMatcher

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class Mapper
{
public:
  /**
   * Default constructor
   */
  Mapper();

  /**
   * Destructor
   */
  virtual ~Mapper();

public:
  /**
   * Allocate memory needed for mapping
   * @param rangeThreshold
   */
  void Initialize(kt_double rangeThreshold);

  /**
   * Resets the mapper.
   * Deallocate memory allocated in Initialize()
   */
  void Reset();

  /**
   * Process a localized range scan for incorporation into the map.  The scan must
   * be identified with a range finder device.  Once added to a map, the corrected pose information in the
   * localized scan will be updated to the correct pose as determined by the mapper.
   *
   * @param pScan A localized range scan that has pose information associated directly with the scan data.  The pose
   * is that of the range device originating the scan.  Note that the mapper will set corrected pose
   * information in the scan object.
   *
   * @return true if the scan was added successfully, false otherwise
   */
  virtual kt_bool Process(LocalizedRangeScan* pScan);

  /**
   * Returns all processed scans added to the mapper.
   * NOTE: The returned scans have their corrected pose updated.
   * @return list of scans received and processed by the mapper. If no scans have been processed,
   * return an empty list.
   */
  virtual const LocalizedRangeScanVector GetAllProcessedScans() const;

  /**
   * Gets the sequential scan matcher
   * @return sequential scan matcher
   */
  virtual ScanMatcher* GetSequentialScanMatcher() const;

  /**
   * Gets the device manager
   * @return device manager
   */
  inline MapperSensorManager* GetMapperSensorManager() const
  {
    return m_pMapperSensorManager;
  }

private:
  void InitializeParameters();

  /**
   * Test if the scan is "sufficiently far" from the last scan added.
   * @param pScan scan to be checked
   * @param pLastScan last scan added to mapper
   * @return true if the scan is "sufficiently far" from the last scan added or
   * the scan is the first scan to be added
   */
  kt_bool HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const;

private:
  /**
   * Restrict the copy constructor
   */
  Mapper(const Mapper&);

  /**
   * Restrict the assignment operator
   */
  const Mapper& operator=(const Mapper&);

public:
  void SetUseScanMatching(kt_bool val)
  {
    m_pUseScanMatching = val;
  }

private:
  kt_bool m_Initialized;

  ScanMatcher* m_pSequentialScanMatcher;

  MapperSensorManager* m_pMapperSensorManager;

  kt_bool m_pUseScanMatching;

  kt_bool m_pUseScanBarycenter;

  kt_double m_pMinimumTravelDistance;

  kt_double m_pMinimumTravelHeading;

  kt_int32u m_pScanBufferSize;

  kt_double m_pScanBufferMaximumScanDistance;

  kt_double m_pCorrelationSearchSpaceDimension;

  kt_double m_pCorrelationSearchSpaceResolution;

  kt_double m_pCorrelationSearchSpaceSmearDeviation;

  kt_double m_pFineSearchAngleOffset;

  kt_double m_pCoarseSearchAngleOffset;

  kt_double m_pCoarseAngleResolution;

  kt_double m_DistanceVariancePenalty;

  kt_double m_AngleVariancePenalty;

  kt_double m_MinimumAnglePenalty;

  kt_double m_MinimumDistancePenalty;

public:
  /* Abstract methods for parameter setters and getters */

  /* Getters */
  // General Parameters
  bool getParamUseScanMatching();
  bool getParamUseScanBarycenter();
  double getParamMinimumTravelDistance();
  double getParamMinimumTravelHeading();
  int getParamScanBufferSize();
  double getParamScanBufferMaximumScanDistance();

  // Correlation Parameters - Correlation Parameters
  double getParamCorrelationSearchSpaceDimension();
  double getParamCorrelationSearchSpaceResolution();
  double getParamCorrelationSearchSpaceSmearDeviation();

  // Scan Matcher Parameters
  double getParamFineSearchAngleOffset();
  double getParamCoarseSearchAngleOffset();
  double getParamCoarseAngleResolution();

  double getParamDistanceVariancePenalty();
  double getParamAngleVariancePenalty();
  double getParamMinimumAnglePenalty();
  double getParamMinimumDistancePenalty();

  /* Setters */
  // General Parameters
  void setParamUseScanMatching(bool b);
  void setParamUseScanBarycenter(bool b);
  void setParamMinimumTravelDistance(double d);
  void setParamMinimumTravelHeading(double d);
  void setParamScanBufferSize(int i);
  void setParamScanBufferMaximumScanDistance(double d);

  // Correlation Parameters - Correlation Parameters
  void setParamCorrelationSearchSpaceDimension(double d);
  void setParamCorrelationSearchSpaceResolution(double d);
  void setParamCorrelationSearchSpaceSmearDeviation(double d);

  // Scan Matcher Parameters
  void setParamFineSearchAngleOffset(double d);
  void setParamCoarseSearchAngleOffset(double d);
  void setParamCoarseAngleResolution(double d);

  void setParamDistanceVariancePenalty(double d);
  void setParamAngleVariancePenalty(double d);
  void setParamMinimumAnglePenalty(double d);
  void setParamMinimumDistancePenalty(double d);
};  // Mapper

}  // namespace KartoScanMatcher

#endif  // KARTO_SCAN_MATCHER_MAPPER_H