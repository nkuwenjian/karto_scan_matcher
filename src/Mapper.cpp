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

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>
#include <iterator>

#include <cmath>
#include <assert.h>

#include "karto_scan_matcher/Mapper.h"

namespace KartoScanMatcher
{
// enable this for verbose debug information
// #define KARTO_DEBUG

#define MAX_VARIANCE 500.0
#define DISTANCE_PENALTY_GAIN 0.2
#define ANGLE_PENALTY_GAIN 0.2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

ScanMatcher::~ScanMatcher()
{
  delete m_pCorrelationGrid;
  delete m_pSearchSpaceProbs;
  delete m_pGridLookup;
}

ScanMatcher* ScanMatcher::Create(Mapper* pMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation,
                                 kt_double rangeThreshold)
{
  // invalid parameters
  if (resolution <= 0)
  {
    return NULL;
  }
  if (searchSize <= 0)
  {
    return NULL;
  }
  if (smearDeviation < 0)
  {
    return NULL;
  }
  if (rangeThreshold <= 0)
  {
    return NULL;
  }

  assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));

  // calculate search space in grid coordinates
  kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);

  // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
  // if a scan is on the border of the search space)
  kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

  kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

  // create correlation grid
  assert(gridSize % 2 == 1);
  CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

  // create search space probabilities
  Grid<kt_double>* pSearchSpaceProbs =
      Grid<kt_double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);

  ScanMatcher* pScanMatcher = new ScanMatcher(pMapper);
  pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
  pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;
  pScanMatcher->m_pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);

  return pScanMatcher;
}

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
kt_double ScanMatcher::MatchScan(LocalizedRangeScan* pScan, const LocalizedRangeScanVector& rBaseScans, Pose2& rMean,
                                 Matrix3& rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
{
  ///////////////////////////////////////
  // set scan pose to be center of grid

  // 1. get scan position
  Pose2 scanPose = pScan->GetSensorPose();

  // scan has no readings; cannot do scan matching
  // best guess of pose is based off of adjusted odometer reading
  if (pScan->GetNumberOfRangeReadings() == 0)
  {
    rMean = scanPose;

    // maximum covariance
    rCovariance(0, 0) = MAX_VARIANCE;                                                  // XX
    rCovariance(1, 1) = MAX_VARIANCE;                                                  // YY
    rCovariance(2, 2) = 4 * math::Square(m_pMapper->getParamCoarseAngleResolution());  // TH*TH

    return 0.0;
  }

  // 2. get size of grid
  Rectangle2<kt_int32s> roi = m_pCorrelationGrid->GetROI();

  // 3. compute offset (in meters - lower left corner)
  Vector2<kt_double> offset;
  offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->GetResolution()));
  offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->GetResolution()));

  // 4. set offset
  m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

  ///////////////////////////////////////

  // set up correlation grid
  AddScans(rBaseScans, scanPose.GetPosition());

  // compute how far to search in each direction
  // searchDimensions = (31, 31)
  Vector2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(), m_pSearchSpaceProbs->GetHeight());
  // coarseSearchOffset = (0.15, 0.15)
  Vector2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->GetResolution(),
                                        0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->GetResolution());

  // a coarse search only checks half the cells in each dimension
  Vector2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->GetResolution(),
                                            2 * m_pCorrelationGrid->GetResolution());

  // actual scan-matching
  kt_double bestResponse = CorrelateScan(
      pScan, scanPose, coarseSearchOffset, coarseSearchResolution, m_pMapper->getParamCoarseSearchAngleOffset(),
      m_pMapper->getParamCoarseAngleResolution(), doPenalize, rMean, rCovariance, false);

  if (doRefineMatch)
  {
    Vector2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
    Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(), m_pCorrelationGrid->GetResolution());
    bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
                                 0.5 * m_pMapper->getParamCoarseAngleResolution(),
                                 m_pMapper->getParamFineSearchAngleOffset(), doPenalize, rMean, rCovariance, true);
  }

  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  return bestResponse;
}

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
kt_double ScanMatcher::CorrelateScan(LocalizedRangeScan* pScan, const Pose2& rSearchCenter,
                                     const Vector2<kt_double>& rSearchSpaceOffset,
                                     const Vector2<kt_double>& rSearchSpaceResolution, kt_double searchAngleOffset,
                                     kt_double searchAngleResolution, kt_bool doPenalize, Pose2& rMean,
                                     Matrix3& rCovariance, kt_bool doingFineMatch)
{
  assert(searchAngleResolution != 0.0);

  // setup lookup arrays
  m_pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);

  // only initialize probability grid if computing positional covariance (during coarse match)
  if (!doingFineMatch)
  {
    m_pSearchSpaceProbs->Clear();

    // position search grid - finds lower left corner of search grid
    Vector2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
    m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
  }

  // calculate position arrays

  std::vector<kt_double> xPoses;
  kt_int32u nX =
      static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
  kt_double startX = -rSearchSpaceOffset.GetX();
  for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
  {
    xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
  }
  assert(math::DoubleEqual(xPoses.back(), -startX));

  std::vector<kt_double> yPoses;
  kt_int32u nY =
      static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
  kt_double startY = -rSearchSpaceOffset.GetY();
  for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
  {
    yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
  }
  assert(math::DoubleEqual(yPoses.back(), -startY));

  // calculate pose response array size
  kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

  kt_int32u poseResponseSize = static_cast<kt_int32u>(xPoses.size() * yPoses.size() * nAngles);

  // allocate array
  std::pair<kt_double, Pose2>* pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];

  Vector2<kt_int32s> startGridPoint =
      m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));

  kt_int32u poseResponseCounter = 0;
  forEachAs(std::vector<kt_double>, &yPoses, yIter)
  {
    kt_double y = *yIter;
    kt_double newPositionY = rSearchCenter.GetY() + y;
    kt_double squareY = math::Square(y);

    forEachAs(std::vector<kt_double>, &xPoses, xIter)
    {
      kt_double x = *xIter;
      kt_double newPositionX = rSearchCenter.GetX() + x;
      kt_double squareX = math::Square(x);

      Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(newPositionX, newPositionY));
      kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
      assert(gridIndex >= 0);

      kt_double angle = 0.0;
      kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
      for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * searchAngleResolution;

        kt_double response = GetResponse(angleIndex, gridIndex);
        if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
        {
          // simple model (approximate Gaussian) to take odometry into account

          kt_double squaredDistance = squareX + squareY;
          kt_double distancePenalty =
              1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_pMapper->getParamDistanceVariancePenalty());
          distancePenalty = math::Maximum(distancePenalty, m_pMapper->getParamMinimumDistancePenalty());

          kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
          kt_double anglePenalty =
              1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_pMapper->getParamAngleVariancePenalty());
          anglePenalty = math::Maximum(anglePenalty, m_pMapper->getParamMinimumAnglePenalty());

          response *= (distancePenalty * anglePenalty);
        }

        // store response and pose
        pPoseResponse[poseResponseCounter] =
            std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
        poseResponseCounter++;
      }

      assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    }
  }

  assert(poseResponseSize == poseResponseCounter);

  // find value of best response (in [0; 1])
  kt_double bestResponse = -1;
  for (kt_int32u i = 0; i < poseResponseSize; i++)
  {
    bestResponse = math::Maximum(bestResponse, pPoseResponse[i].first);

    // will compute positional covariance, save best relative probability for each cell
    if (!doingFineMatch)
    {
      const Pose2& rPose = pPoseResponse[i].second;
      Vector2<kt_int32s> grid = m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());

      // Changed (kt_double*) to the reinterpret_cast - Luc
      kt_double* ptr = reinterpret_cast<kt_double*>(m_pSearchSpaceProbs->GetDataPointer(grid));
      if (ptr == NULL)
      {
        throw std::runtime_error("Mapper FATAL ERROR - Index out of range in probability search!");
      }

      *ptr = math::Maximum(pPoseResponse[i].first, *ptr);
    }
  }

  // average all poses with same highest response
  Vector2<kt_double> averagePosition;
  kt_double thetaX = 0.0;
  kt_double thetaY = 0.0;
  kt_int32s averagePoseCount = 0;
  for (kt_int32u i = 0; i < poseResponseSize; i++)
  {
    if (math::DoubleEqual(pPoseResponse[i].first, bestResponse))
    {
      averagePosition += pPoseResponse[i].second.GetPosition();

      kt_double heading = pPoseResponse[i].second.GetHeading();
      thetaX += cos(heading);
      thetaY += sin(heading);

      averagePoseCount++;
    }
  }

  Pose2 averagePose;
  if (averagePoseCount > 0)
  {
    averagePosition /= averagePoseCount;

    thetaX /= averagePoseCount;
    thetaY /= averagePoseCount;

    averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
  }
  else
  {
    throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
  }

  // delete pose response array
  delete[] pPoseResponse;

  rMean = averagePose;

  if (bestResponse > 1.0)
  {
    bestResponse = 1.0;
  }

  assert(math::InRange(bestResponse, 0.0, 1.0));
  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  return bestResponse;
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint)
{
  m_pCorrelationGrid->Clear();

  // add all scans to grid
  const_forEach(LocalizedRangeScanVector, &rScans)
  {
    AddScan(*iter, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear)
{
  PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

  // put in all valid points
  const_forEach(PointVectorDouble, &validPoints)
  {
    Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(*iter);
    if (!math::IsUpTo(gridPoint.GetX(), m_pCorrelationGrid->GetROI().GetWidth()) ||
        !math::IsUpTo(gridPoint.GetY(), m_pCorrelationGrid->GetROI().GetHeight()))
    {
      // point not in grid
      continue;
    }

    int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    // set grid cell as occupied
    if (m_pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
    {
      // value already set
      continue;
    }

    m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

    // smear grid
    if (doSmear == true)
    {
      m_pCorrelationGrid->SmearPoint(gridPoint);
    }
  }
}

/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param pScan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble ScanMatcher::FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const
{
  const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

  // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
  const kt_double minSquareDistance = math::Square(0.1);  // in m^2

  // this iterator lags from the main iterator adding points only when the points are on
  // the same side as the viewpoint
  PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
  PointVectorDouble validPoints;

  Vector2<kt_double> firstPoint;
  kt_bool firstTime = true;
  const_forEach(PointVectorDouble, &rPointReadings)
  {
    Vector2<kt_double> currentPoint = *iter;

    if (firstTime && !std::isnan(currentPoint.GetX()) && !std::isnan(currentPoint.GetY()))
    {
      firstPoint = currentPoint;
      firstTime = false;
    }

    Vector2<kt_double> delta = firstPoint - currentPoint;
    if (delta.SquaredLength() > minSquareDistance)
    {
      // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
      // Which computes the direction of rotation, if the rotation is counterclock
      // wise then we are looking at data we should keep. If it's negative rotation
      // we should not included in in the matching
      // have enough distance, check viewpoint
      double a = rViewPoint.GetY() - firstPoint.GetY();
      double b = firstPoint.GetX() - rViewPoint.GetX();
      double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
      double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

      // reset beginning point
      firstPoint = currentPoint;

      if (ss < 0.0)  // wrong side, skip and keep going
      {
        trailingPointIter = iter;
      }
      else
      {
        for (; trailingPointIter != iter; ++trailingPointIter)
        {
          validPoints.push_back(*trailingPointIter);
        }
      }
    }
  }

  return validPoints;
}

/**
 * Get response at given position for given rotation (only look up valid points)
 * @param angleIndex
 * @param gridPositionIndex
 * @return response
 */
kt_double ScanMatcher::GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const
{
  kt_double response = 0.0;

  // add up value for each point
  kt_int8u* pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

  const LookupArray* pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
  assert(pOffsets != NULL);

  // get number of points in offset list
  kt_int32u nPoints = pOffsets->GetSize();
  if (nPoints == 0)
  {
    return response;
  }

  // calculate response
  kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
  for (kt_int32u i = 0; i < nPoints; i++)
  {
    // ignore points that fall off the grid
    kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
    // assert(math::IsUpTo(pointGridIndex, m_pCorrelationGrid->GetDataSize()));
    if (!math::IsUpTo(pointGridIndex, m_pCorrelationGrid->GetDataSize()) || pAngleIndexPointer[i] == INVALID_SCAN)
    {
      continue;
    }

    // uses index offsets to efficiently find location of point in the grid
    response += pByte[pAngleIndexPointer[i]];
  }

  // normalize response
  response /= (nPoints * GridStates_Occupied);
  assert(fabs(response) <= 1.0);

  return response;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Default constructor
 */
Mapper::Mapper() : m_Initialized(false), m_pSequentialScanMatcher(NULL), m_pMapperSensorManager(NULL)
{
  InitializeParameters();
}

/**
 * Destructor
 */
Mapper::~Mapper()
{
  Reset();

  delete m_pMapperSensorManager;
}

void Mapper::InitializeParameters()
{
  m_pUseScanMatching = true;
  m_pUseScanBarycenter = true;
  m_pMinimumTravelDistance = 0.2;
  m_pMinimumTravelHeading = math::DegreesToRadians(10);
  m_pScanBufferSize = 70;
  m_pScanBufferMaximumScanDistance = 20.0;

  //////////////////////////////////////////////////////////////////////////////
  //    CorrelationParameters correlationParameters;
  m_pCorrelationSearchSpaceDimension = 0.3;
  m_pCorrelationSearchSpaceResolution = 0.01;
  m_pCorrelationSearchSpaceSmearDeviation = 0.03;

  //////////////////////////////////////////////////////////////////////////////
  // ScanMatcherParameters;
  m_pFineSearchAngleOffset = math::DegreesToRadians(0.2);
  m_pCoarseSearchAngleOffset = math::DegreesToRadians(20);
  m_pCoarseAngleResolution = math::DegreesToRadians(2);

  m_DistanceVariancePenalty = math::Square(0.3);
  m_AngleVariancePenalty = math::Square(math::DegreesToRadians(20));
  m_MinimumAnglePenalty = 0.9;
  m_MinimumDistancePenalty = 0.5;
}
/* Adding in getters and setters here for easy parameter access */

// General Parameters

bool Mapper::getParamUseScanMatching()
{
  return static_cast<bool>(m_pUseScanMatching);
}

bool Mapper::getParamUseScanBarycenter()
{
  return static_cast<bool>(m_pUseScanBarycenter);
}

double Mapper::getParamMinimumTravelDistance()
{
  return static_cast<double>(m_pMinimumTravelDistance);
}

double Mapper::getParamMinimumTravelHeading()
{
  return math::RadiansToDegrees(static_cast<double>(m_pMinimumTravelHeading));
}

int Mapper::getParamScanBufferSize()
{
  return static_cast<int>(m_pScanBufferSize);
}

double Mapper::getParamScanBufferMaximumScanDistance()
{
  return static_cast<double>(m_pScanBufferMaximumScanDistance);
}

// Correlation Parameters - Correlation Parameters

double Mapper::getParamCorrelationSearchSpaceDimension()
{
  return static_cast<double>(m_pCorrelationSearchSpaceDimension);
}

double Mapper::getParamCorrelationSearchSpaceResolution()
{
  return static_cast<double>(m_pCorrelationSearchSpaceResolution);
}

double Mapper::getParamCorrelationSearchSpaceSmearDeviation()
{
  return static_cast<double>(m_pCorrelationSearchSpaceSmearDeviation);
}

// ScanMatcher Parameters

double Mapper::getParamFineSearchAngleOffset()
{
  return static_cast<double>(m_pFineSearchAngleOffset);
}

double Mapper::getParamCoarseSearchAngleOffset()
{
  return static_cast<double>(m_pCoarseSearchAngleOffset);
}

double Mapper::getParamCoarseAngleResolution()
{
  return static_cast<double>(m_pCoarseAngleResolution);
}

double Mapper::getParamDistanceVariancePenalty()
{
  return static_cast<double>(m_DistanceVariancePenalty);
}

double Mapper::getParamAngleVariancePenalty()
{
  return static_cast<double>(m_AngleVariancePenalty);
}

double Mapper::getParamMinimumAnglePenalty()
{
  return static_cast<double>(m_MinimumAnglePenalty);
}

double Mapper::getParamMinimumDistancePenalty()
{
  return static_cast<double>(m_MinimumDistancePenalty);
}

/* Setters for parameters */
// General Parameters
void Mapper::setParamUseScanMatching(bool b)
{
  m_pUseScanMatching = (kt_bool)b;
}

void Mapper::setParamUseScanBarycenter(bool b)
{
  m_pUseScanBarycenter = (kt_bool)b;
}

void Mapper::setParamMinimumTravelDistance(double d)
{
  m_pMinimumTravelDistance = (kt_double)d;
}

void Mapper::setParamMinimumTravelHeading(double d)
{
  m_pMinimumTravelHeading = (kt_double)d;
}

void Mapper::setParamScanBufferSize(int i)
{
  m_pScanBufferSize = (kt_int32u)i;
}

void Mapper::setParamScanBufferMaximumScanDistance(double d)
{
  m_pScanBufferMaximumScanDistance = (kt_double)d;
}

// Correlation Parameters - Correlation Parameters
void Mapper::setParamCorrelationSearchSpaceDimension(double d)
{
  m_pCorrelationSearchSpaceDimension = (kt_double)d;
}

void Mapper::setParamCorrelationSearchSpaceResolution(double d)
{
  m_pCorrelationSearchSpaceResolution = (kt_double)d;
}

void Mapper::setParamCorrelationSearchSpaceSmearDeviation(double d)
{
  m_pCorrelationSearchSpaceSmearDeviation = (kt_double)d;
}

// Scan Matcher Parameters

void Mapper::setParamFineSearchAngleOffset(double d)
{
  m_pFineSearchAngleOffset = (kt_double)d;
}

void Mapper::setParamCoarseSearchAngleOffset(double d)
{
  m_pCoarseSearchAngleOffset = (kt_double)d;
}

void Mapper::setParamCoarseAngleResolution(double d)
{
  m_pCoarseAngleResolution = (kt_double)d;
}

void Mapper::setParamDistanceVariancePenalty(double d)
{
  m_DistanceVariancePenalty = (kt_double)d;
}

void Mapper::setParamAngleVariancePenalty(double d)
{
  m_AngleVariancePenalty = (kt_double)d;
}

void Mapper::setParamMinimumAnglePenalty(double d)
{
  m_MinimumAnglePenalty = (kt_double)d;
}

void Mapper::setParamMinimumDistancePenalty(double d)
{
  m_MinimumDistancePenalty = (kt_double)d;
}

void Mapper::Initialize(kt_double rangeThreshold)
{
  if (m_Initialized == false)
  {
    m_pSequentialScanMatcher =
        ScanMatcher::Create(this, m_pCorrelationSearchSpaceDimension, m_pCorrelationSearchSpaceResolution,
                            m_pCorrelationSearchSpaceSmearDeviation, rangeThreshold);
    assert(m_pSequentialScanMatcher);

    m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize, m_pScanBufferMaximumScanDistance);

    m_Initialized = true;
  }
}

void Mapper::Reset()
{
  delete m_pSequentialScanMatcher;
  m_pSequentialScanMatcher = NULL;

  m_Initialized = false;
}

kt_bool Mapper::Process(LocalizedRangeScan* pScan)
{
  if (pScan != NULL)
  {
    LaserRangeFinder* pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false)
    {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // get last scan
    LocalizedRangeScan* pLastScan = m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL)
    {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!HasMovedEnough(pScan, pLastScan))
    {
      return false;
    }

    Matrix3 covariance;
    covariance.SetToIdentity();

    // scan matching
    if (m_pUseScanMatching && pLastScan != NULL)
    {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan, m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
                                          bestPose, covariance);
      pScan->SetSensorPose(bestPose);
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching)
    {
      m_pMapperSensorManager->AddRunningScan(pScan);
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

/**
 * Is the scan sufficiently far from the last scan?
 * @param pScan
 * @param pLastScan
 * @return true if the scans are sufficiently far
 */
kt_bool Mapper::HasMovedEnough(LocalizedRangeScan* pScan, LocalizedRangeScan* pLastScan) const
{
  // test if first scan
  if (pLastScan == NULL)
  {
    return true;
  }

  Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  // test if we have turned enough
  kt_double deltaHeading = math::NormalizeAngle(scannerPose.GetHeading() - lastScannerPose.GetHeading());
  if (fabs(deltaHeading) >= m_pMinimumTravelHeading)
  {
    return true;
  }

  // test if we have moved enough
  kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(scannerPose.GetPosition());
  if (squaredTravelDistance >= math::Square(m_pMinimumTravelDistance) - KT_TOLERANCE)
  {
    return true;
  }

  return false;
}

/**
 * Gets all the processed scans
 * @return all scans
 */
const LocalizedRangeScanVector Mapper::GetAllProcessedScans() const
{
  LocalizedRangeScanVector allScans;

  if (m_pMapperSensorManager != NULL)
  {
    allScans = m_pMapperSensorManager->GetAllScans();
  }

  return allScans;
}

ScanMatcher* Mapper::GetSequentialScanMatcher() const
{
  return m_pSequentialScanMatcher;
}

}  // namespace KartoScanMatcher