#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>
#include <iterator>

#include <math.h>
#include <assert.h>

#include "karto_scan_matcher/Mapper.h"

namespace KartoScanMatcher
{

	/**
	 * Default constructor
	 */
	Mapper::Mapper()
		: m_Initialized(false)
		, m_pSequentialScanMatcher(NULL)
		, m_pMapperSensorManager(NULL)
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

	// 设置基本参数
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


	void Mapper::Initialize(kt_double rangeThreshold)
	{
		if (m_Initialized == false)
		{
			// create sequential scan and loop matcher
			m_pSequentialScanMatcher = ScanMatcher::Create(m_pCorrelationSearchSpaceDimension,
														m_pCorrelationSearchSpaceResolution,
														m_pCorrelationSearchSpaceSmearDeviation,
														rangeThreshold,
														m_pFineSearchAngleOffset,
														m_pCoarseSearchAngleOffset,
														m_pCoarseAngleResolution);
			assert(m_pSequentialScanMatcher);

			m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize,
														m_pScanBufferMaximumScanDistance);

			m_Initialized = true;
		}
	}

	void Mapper::Reset()
	{
		delete m_pSequentialScanMatcher;
		m_pSequentialScanMatcher = NULL;

		delete m_pMapperSensorManager;
		m_pMapperSensorManager = NULL;

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

			// correct scan (if not first scan)
			if (m_pUseScanMatching && pLastScan != NULL)
			{
				Pose2 bestPose;
				// 核心一：扫描匹配
				m_pSequentialScanMatcher->MatchScan(pScan,
													m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
																							bestPose,
																							covariance);
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
