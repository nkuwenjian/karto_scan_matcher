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

#ifndef _MAPPER_H_
#define _MAPPER_H_

#include <map>
#include <vector>

#include <karto_scan_matcher/ScanMatcher.h>
#include <karto_scan_matcher/MapperSensorManager.h>

namespace KartoScanMatcher
{
/**
 * Graph SLAM mapper. Creates a map given a set of LocalizedRangeScans
 * The current Karto implementation is a proprietary, high-performance
 * scan-matching algorithm that constructs a map from individual range scans and corrects for
 * errors in the range and odometry data.
 *
 * The following parameters can be set on the Mapper.
 *
 *  \a UseScanMatching (ParameterBool)\n
 *     When set to true, the mapper will use a scan matching algorithm. In most real-world situations
 *     this should be set to true so that the mapper algorithm can correct for noise and errors in
 *     odometry and scan data. In some simulator environments where the simulated scan and odometry
 *     data are very accurate, the scan matching algorithm can produce worse results. In those cases
 *     set to false to improve results.
 *     Default value is true.
 *
 *  \a UseScanBarycenter (ParameterBool)\n
 *
 *  \a UseResponseExpansion (ParameterBool)\n
 *
 *  \a RangeThreshold (ParameterDouble - meters)\n
 *     The range threshold is used to truncate range scan distance measurement readings.  The threshold should
 *     be set such that adjacent range readings in a scan will generally give "solid" coverage of objects.
 *
 *     \image html doxygen/RangeThreshold.png
 *     \image latex doxygen/RangeThreshold.png "" width=3in
 *
 *     Having solid coverage depends on the map resolution and the angular resolution of the range scan device.
 *     The following are the recommended threshold values for the corresponding map resolution and range finder
 *     resolution values:
 *
 *     <table border=0>
 *      <tr>
 *       <td><b>Map Resolution</b></td>
 *       <td colspan=3 align=center><b>Laser Angular Resolution</b></td>
 *      </tr>
 *      <tr>
 *       <td></td>
 *       <td align=center><b>1.0 degree</b></td>
 *       <td align=center><b>0.5 degree</b></td>
 *       <td align=center><b>0.25 degree</b></td>
 *      </tr>
 *      <tr>
 *       <td align=center><b>0.1</b></td>
 *       <td align=center>5.7m</td>
 *       <td align=center>11.4m</td>
 *       <td align=center>22.9m</td>
 *      </tr>
 *      <tr>
 *       <td align=center><b>0.05</b></td>
 *       <td align=center>2.8m</td>
 *       <td align=center>5.7m</td>
 *       <td align=center>11.4m</td>
 *      </tr>
 *      <tr>
 *       <td align=center><b>0.01</b></td>
 *       <td align=center>0.5m</td>
 *       <td align=center>1.1m</td>
 *       <td align=center>2.3m</td>
 *      </tr>
 *     </table>
 *
 *     Note that the value of RangeThreshold should be adjusted taking into account the values of
 *     MinimumTravelDistance and MinimumTravelHeading (see also below).  By incorporating scans
 *     into the map more frequently, the RangeThreshold value can be increased as the additional scans
 *     will "fill in" the gaps of objects at greater distances where there is less solid coverage.
 *
 *     Default value is 12.0 (meters).
 *
 *  \a MinimumTravelDistance (ParameterDouble - meters)\n
 *     Sets the minimum travel between scans. If a new scan's position is more than minimumDistance from
 *     the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
 *     new scan if it also does not meet the minimum change in heading requirement.
 *     For performance reasons, generally it is a good idea to only process scans if the robot
 *     has moved a reasonable amount.
 *     Default value is 0.3 (meters).
 *
 *  \a MinimumTravelHeading (ParameterDouble - radians)\n
 *     Sets the minimum heading change between scans. If a new scan's heading is more than minimumHeading from
 *     the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
 *     new scan if it also does not meet the minimum travel distance requirement.
 *     For performance reasons, generally it is a good idea to only process scans if the robot
 *     has moved a reasonable amount.
 *     Default value is 0.08726646259971647 (radians) - 5 degrees.
 *
 *  \a ScanBufferSize (ParameterIn32u - size)\n
 *     Scan buffer size is the length of the scan chain stored for scan matching.
 *     "ScanBufferSize" should be set to approximately "ScanBufferMaximumScanDistance" / "MinimumTravelDistance".
 *     The idea is to get an area approximately 20 meters long for scan matching.
 *     For example, if we add scans every MinimumTravelDistance = 0.3 meters, then "ScanBufferSize"
 *     should be 20 / 0.3 = 67.)
 *     Default value is 67.
 *
 *  \a ScanBufferMaximumScanDistance (ParameterDouble - meters)\n
 *     Scan buffer maximum scan distance is the maximum distance between the first and last scans
 *     in the scan chain stored for matching.
 *     Default value is 20.0.
 *
 *  \a CorrelationSearchSpaceDimension (ParameterDouble - meters)\n
 *     The size of the correlation grid used by the matcher.
 *     Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
 *
 *  \a CorrelationSearchSpaceResolution (ParameterDouble - meters)\n
 *     The resolution (size of a grid cell) of the correlation grid.
 *     Default value is 0.01 meters.
 *
 *  \a CorrelationSearchSpaceSmearDeviation (ParameterDouble - meters)\n
 *     The robot position is smeared by this value in X and Y to create a smoother response.
 *     Default value is 0.03 meters.
 *
 *  \a LinkMatchMinimumResponseFine (ParameterDouble - probability (>= 0.0, <= 1.0))\n
 *     Scans are linked only if the correlation response value is greater than this value.
 *     Default value is 0.4
 *
 *  \a LinkScanMaximumDistance (ParameterDouble - meters)\n
 *     Maximum distance between linked scans.  Scans that are farther apart will not be linked
 *     regardless of the correlation response value.
 *     Default value is 6.0 meters.
 *
 *  \a LoopSearchSpaceDimension (ParameterDouble - meters)\n
 *     Dimension of the correlation grid used by the loop closure detection algorithm
 *     Default value is 4.0 meters.
 *
 *  \a LoopSearchSpaceResolution (ParameterDouble - meters)\n
 *     Coarse resolution of the correlation grid used by the matcher to determine a possible
 *     loop closure.
 *     Default value is 0.05 meters.
 *
 *  \a LoopSearchSpaceSmearDeviation (ParameterDouble - meters)\n
 *     Smearing distance in the correlation grid used by the matcher to determine a possible
 *     loop closure match.
 *     Default value is 0.03 meters.
 *
 *  \a LoopSearchMaximumDistance (ParameterDouble - meters)\n
 *     Scans less than this distance from the current position will be considered for a match
 *     in loop closure.
 *     Default value is 4.0 meters.
 *
 *  \a LoopMatchMinimumChainSize (ParameterIn32s)\n
 *     When the loop closure detection finds a candidate it must be part of a large
 *     set of linked scans. If the chain of scans is less than this value we do not attempt
 *     to close the loop.
 *     Default value is 10.
 *
 *  \a LoopMatchMaximumVarianceCoarse (ParameterDouble)\n
 *     The co-variance values for a possible loop closure have to be less than this value
 *     to consider a viable solution. This applies to the coarse search.
 *     Default value is 0.16.
 *
 *  \a LoopMatchMinimumResponseCoarse (ParameterDouble - probability (>= 0.0, <= 1.0))\n
 *     If response is larger then this then initiate loop closure search at the coarse resolution.
 *     Default value is 0.7.
 *
 *  \a LoopMatchMinimumResponseFine (ParameterDouble - probability (>= 0.0, <= 1.0))\n
 *     If response is larger then this then initiate loop closure search at the fine resolution.
 *     Default value is 0.5.
 */
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
	void SetUseScanMatching(kt_bool val) { m_pUseScanMatching = val; }

private:
	kt_bool m_Initialized;

	ScanMatcher* m_pSequentialScanMatcher;

	MapperSensorManager* m_pMapperSensorManager;


	/**
	 * When set to true, the mapper will use a scan matching algorithm. In most real-world situations
	 * this should be set to true so that the mapper algorithm can correct for noise and errors in
	 * odometry and scan data. In some simulator environments where the simulated scan and odometry
	 * data are very accurate, the scan matching algorithm can produce worse results. In those cases
	 * set this to false to improve results.
	 * Default value is true.
	 */
	kt_bool m_pUseScanMatching;

	/**
	 * Default value is true.
	 */
	kt_bool m_pUseScanBarycenter;

	/**
	 * Sets the minimum travel between scans.  If a new scan's position is more than minimumTravelDistance
	 * from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
	 * new scan if it also does not meet the minimum change in heading requirement.
	 * For performance reasons, generally it is a good idea to only process scans if the robot
	 * has moved a reasonable amount.
	 * Default value is 0.2 (meters).
	 */
	kt_double m_pMinimumTravelDistance;

	/**
	 * Sets the minimum heading change between scans. If a new scan's heading is more than minimumTravelHeading
	 * from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
	 * new scan if it also does not meet the minimum travel distance requirement.
	 * For performance reasons, generally it is a good idea to only process scans if the robot
	 * has moved a reasonable amount.
	 * Default value is 10 degrees.
	 */
	kt_double m_pMinimumTravelHeading;

	/**
	 * Scan buffer size is the length of the scan chain stored for scan matching.
	 * "scanBufferSize" should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
	 * The idea is to get an area approximately 20 meters long for scan matching.
	 * For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
	 * should be 20 / 0.3 = 67.)
	 * Default value is 67.
	 */
	kt_int32u m_pScanBufferSize;

	/**
	 * Scan buffer maximum scan distance is the maximum distance between the first and last scans
	 * in the scan chain stored for matching.
	 * Default value is 20.0.
	 */
	kt_double m_pScanBufferMaximumScanDistance;


	//////////////////////////////////////////////////////////////////////////////
	//    CorrelationParameters correlationParameters;

	/**
	 * The size of the search grid used by the matcher.
	 * Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
	 */
	kt_double m_pCorrelationSearchSpaceDimension;

	/**
	 * The resolution (size of a grid cell) of the correlation grid.
	 * Default value is 0.01 meters.
	 */
	kt_double m_pCorrelationSearchSpaceResolution;

	/**
	 * The point readings are smeared by this value in X and Y to create a smoother response.
	 * Default value is 0.03 meters.
	 */
	kt_double m_pCorrelationSearchSpaceSmearDeviation;


	//////////////////////////////////////////////////////////////////////////////
	// ScanMatcherParameters;

	// The range of angles to search during a coarse search and a finer search
	kt_double m_pFineSearchAngleOffset;
	kt_double m_pCoarseSearchAngleOffset;

	// Resolution of angles to search during a coarse search
	kt_double m_pCoarseAngleResolution;

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
};  // Mapper

}  // namespace KartoScanMatcher

#endif  // _MAPPER_H_
