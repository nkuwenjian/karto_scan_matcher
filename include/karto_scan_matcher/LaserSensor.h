#ifndef _LASER_SENSOR_H_
#define _LASER_SENSOR_H_

#include <karto_scan_matcher/DataStructure.h>
#include <karto_scan_matcher/Macros.h>
#include <karto_scan_matcher/Math.h>
#include <karto_scan_matcher/CoordinateConverter.h>

#include <iostream>
#include <boost/thread.hpp>

namespace KartoScanMatcher
{
/**
 * Type declaration of range readings vector
 */
typedef std::vector<kt_double> RangeReadingsVector;


/**
 * LaserRangeScan representing the range readings from a laser range finder sensor.
 */
class LaserRangeScan
{
public:
  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   */
  LaserRangeScan(const std::string& rSensorName)
  : m_SensorName(rSensorName)
  , m_pRangeReadings(NULL)
  , m_NumberOfRangeReadings(0)
  {
  }

  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   * @param rRangeReadings
   */
  LaserRangeScan(const std::string& rSensorName, const RangeReadingsVector& rRangeReadings)
  : m_SensorName(rSensorName)
  , m_pRangeReadings(NULL)
  , m_NumberOfRangeReadings(0)
  {
    assert(rSensorName != "");

    SetRangeReadings(rRangeReadings);
  }

  /**
   * Destructor
   */
  virtual ~LaserRangeScan()
  {
    delete [] m_pRangeReadings;
  }

public:
  /**
   * Get the sensor that created this sensor data
   * @return sensor
   */
  inline const std::string& GetSensorName() const
  {
    return m_SensorName;
  }

  /**
   * Gets the range readings of this scan
   * @return range readings of this scan
   */
  inline kt_double* GetRangeReadings() const
  {
    return m_pRangeReadings;
  }

  inline RangeReadingsVector GetRangeReadingsVector() const
  {
    return RangeReadingsVector(m_pRangeReadings, m_pRangeReadings + m_NumberOfRangeReadings);
  }

  /**
   * Sets the range readings for this scan
   * @param rRangeReadings
   */
  inline void SetRangeReadings(const RangeReadingsVector& rRangeReadings)
  {
    if (!rRangeReadings.empty())
    {
      if (rRangeReadings.size() != m_NumberOfRangeReadings)
      {
        // delete old readings
        delete [] m_pRangeReadings;

        // store size of array!
        m_NumberOfRangeReadings = static_cast<kt_int32u>(rRangeReadings.size());

        // allocate range readings
        m_pRangeReadings = new kt_double[m_NumberOfRangeReadings];
      }

      // copy readings
      kt_int32u index = 0;
      const_forEach(RangeReadingsVector, &rRangeReadings)
      {
        m_pRangeReadings[index++] = *iter;
      }
    }
    else
    {
      delete [] m_pRangeReadings;
      m_pRangeReadings = NULL;
    }
  }

  /**
   * Gets the number of range readings
   * @return number of range readings
   */
  inline kt_int32u GetNumberOfRangeReadings() const
  {
    return m_NumberOfRangeReadings;
  }

private:
  LaserRangeScan(const LaserRangeScan&);
  const LaserRangeScan& operator=(const LaserRangeScan&);

private:
  kt_double* m_pRangeReadings;
  kt_int32u m_NumberOfRangeReadings;
  /**
   * Sensor that created this sensor data
   */
  std::string m_SensorName;
};  // LaserRangeScan


/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset position of a localized range scan relative to the robot.
 * The user can set an offset pose for the sensor relative to the robot coordinate system. If no value is provided
 * by the user, the sensor is set to be at the origin of the robot coordinate system.
 * The LaserRangeFinder contains parameters for physical laser sensor used by the mapper for scan matching
 * Also contains information about the maximum range of the sensor and provides a threshold
 * for limiting the range of readings.
 * The optimal value for the range threshold depends on the angular resolution of the scan and
 * the desired map resolution.  RangeThreshold should be set as large as possible while still
 * providing "solid" coverage between consecutive range readings.  The diagram below illustrates
 * the relationship between map resolution and the range threshold.
 */
class LaserRangeFinder
{
public:
  /**
   * Destructor
   */
  virtual ~LaserRangeFinder()
  {
  }

public:
  /**
   * Gets this range finder sensor's offset
   * @return offset pose
   */
  inline const Pose2& GetOffsetPose() const
  {
    return m_pOffsetPose;
  }

  /**
   * Sets this range finder sensor's offset
   * @param rPose
   */
  inline void SetOffsetPose(const Pose2& rPose)
  {
    m_pOffsetPose = rPose;
  }

  /**
   * Gets the name of this object
   * @return name
   */
  inline const std::string& GetName() const
  {
    return m_Name;
  }

  /**
   * Gets this range finder sensor's minimum range
   * @return minimum range
   */
  inline kt_double GetMinimumRange() const
  {
    return m_pMinimumRange;
  }

  /**
   * Sets this range finder sensor's minimum range
   * @param minimumRange
   */
  inline void SetMinimumRange(kt_double minimumRange)
  {
    m_pMinimumRange = minimumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets this range finder sensor's maximum range
   * @return maximum range
   */
  inline kt_double GetMaximumRange() const
  {
    return m_pMaximumRange;
  }

  /**
   * Sets this range finder sensor's maximum range
   * @param maximumRange
   */
  inline void SetMaximumRange(kt_double maximumRange)
  {
    m_pMaximumRange = maximumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets the range threshold
   * @return range threshold
   */
  inline kt_double GetRangeThreshold() const
  {
    return m_pRangeThreshold;
  }

  /**
   * Sets the range threshold
   * @param rangeThreshold
   */
  inline void SetRangeThreshold(kt_double rangeThreshold)
  {
    // make sure rangeThreshold is within laser range finder range
    m_pRangeThreshold = math::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange());

    if (math::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false)
    {
      std::cout << "Info: clipped range threshold to be within minimum and maximum range!" << std::endl;
    }
  }

  /**
   * Gets this range finder sensor's minimum angle
   * @return minimum angle
   */
  inline kt_double GetMinimumAngle() const
  {
    return m_pMinimumAngle;
  }

  /**
   * Sets this range finder sensor's minimum angle
   * @param minimumAngle
   */
  inline void SetMinimumAngle(kt_double minimumAngle)
  {
    m_pMinimumAngle = minimumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's maximum angle
   * @return maximum angle
   */
  inline kt_double GetMaximumAngle() const
  {
    return m_pMaximumAngle;
  }

  /**
   * Sets this range finder sensor's maximum angle
   * @param maximumAngle
   */
  inline void SetMaximumAngle(kt_double maximumAngle)
  {
    m_pMaximumAngle = maximumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's angular resolution
   * @return angular resolution
   */
  inline kt_double GetAngularResolution() const
  {
    return m_pAngularResolution;
  }

  /**
   * Sets this range finder sensor's angular resolution
   * @param angularResolution
   */
  inline void SetAngularResolution(kt_double angularResolution)
  {
    m_pAngularResolution = angularResolution;

    Update();
  }

  /**
   * Gets the number of range readings each localized range scan must contain to be a valid scan.
   * @return number of range readings
   */
  inline kt_int32u GetNumberOfRangeReadings() const
  {
    return m_NumberOfRangeReadings;
  }

  virtual kt_bool Validate()
  {
    Update();

    if (math::InRange(GetRangeThreshold(), GetMinimumRange(), GetMaximumRange()) == false)
    {
      std::cout << "Please set range threshold to a value between ["
          << GetMinimumRange() << ";" << GetMaximumRange() << "]" << std::endl;
      return false;
    }

    return true;
  }

  virtual kt_bool Validate(LaserRangeScan* pLaserRangeScan)
  {
    // verify number of range readings in LaserRangeScan matches the number of expected range readings
    if (pLaserRangeScan->GetNumberOfRangeReadings() != GetNumberOfRangeReadings())
    {
      std::cout << "LaserRangeScan contains " << pLaserRangeScan->GetNumberOfRangeReadings()
        << " range readings, expected " << GetNumberOfRangeReadings() << std::endl;
      return false;
    }
    return true;
  }


public:
  /**
   * Create a laser range finder of the given type and ID
   * @param type
   * @param rName name of sensor - if no name is specified default name will be assigned
   * @return laser range finder
   */
  LaserRangeFinder* CreateLaserRangeFinder(LaserRangeFinderType type, const std::string& rName)
  {
    LaserRangeFinder* pLrf = NULL;

    switch (type)
    {
      // see http://www.hizook.com/files/publications/SICK_LMS100.pdf
      // set range threshold to 18m
      case LaserRangeFinder_Sick_LMS100:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("Sick LMS 100"));

        // Sensing range is 18 meters (at 10% reflectivity, max range of 20 meters), with an error of about 20mm
        pLrf->m_pMinimumRange = 0.0;
        pLrf->m_pMaximumRange = 20.0;

        // 270 degree range, 50 Hz
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-135);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(135);

        // 0.25 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(0.25);

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 10m
      case LaserRangeFinder_Sick_LMS200:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("Sick LMS 200"));

        // Sensing range is 80 meters
        pLrf->m_pMinimumRange = 0.0;
        pLrf->m_pMaximumRange = 80.0;

        // 180 degree range, 75 Hz
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-90);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(90);

        // 0.5 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(0.5);

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/SICK_LMS200-291_Tech_Info.pdf
      // set range threshold to 30m
      case LaserRangeFinder_Sick_LMS291:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("Sick LMS 291"));

        // Sensing range is 80 meters
        pLrf->m_pMinimumRange = 0.0;
        pLrf->m_pMaximumRange = 80.0;

        // 180 degree range, 75 Hz
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-90);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(90);

        // 0.5 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(0.5);

        pLrf->m_NumberOfRangeReadings = 361;
      }
      break;

      // see http://www.hizook.com/files/publications/Hokuyo_UTM_LaserRangeFinder_LIDAR.pdf
      // set range threshold to 30m
      case LaserRangeFinder_Hokuyo_UTM_30LX:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("Hokuyo UTM-30LX"));

        // Sensing range is 30 meters
        pLrf->m_pMinimumRange = 0.1;
        pLrf->m_pMaximumRange = 30.0;

        // 270 degree range, 40 Hz
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-135);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(135);

        // 0.25 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(0.25);

        pLrf->m_NumberOfRangeReadings = 1081;
      }
      break;

      // see http://www.hizook.com/files/publications/HokuyoURG_Datasheet.pdf
      // set range threshold to 3.5m
      case LaserRangeFinder_Hokuyo_URG_04LX:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("Hokuyo URG-04LX"));

        // Sensing range is 4 meters. It has detection problems when
        // scanning absorptive surfaces (such as black trimming).
        pLrf->m_pMinimumRange = 0.02;
        pLrf->m_pMaximumRange = 4.0;

        // 240 degree range, 10 Hz
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-120);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(120);

        // 0.352 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(0.352);

        pLrf->m_NumberOfRangeReadings = 751;
      }
      break;

      case LaserRangeFinder_Custom:
      {
        pLrf = new LaserRangeFinder((rName != "") ? rName : std::string("User-Defined LaserRangeFinder"));

        // Sensing range is 80 meters.
        pLrf->m_pMinimumRange = 0.0;
        pLrf->m_pMaximumRange = 80.0;

        // 180 degree range
        pLrf->m_pMinimumAngle = math::DegreesToRadians(-90);
        pLrf->m_pMaximumAngle = math::DegreesToRadians(90);

        // 1.0 degree angular resolution
        pLrf->m_pAngularResolution = math::DegreesToRadians(1.0);

        pLrf->m_NumberOfRangeReadings = 181;
      }
      break;
    }

    if (pLrf != NULL)
    {
      Pose2 defaultOffset;
      pLrf->SetOffsetPose(defaultOffset);
    }

    return pLrf;
  }

private:
  /**
   * Constructs a LaserRangeFinder object with given ID
   */
  LaserRangeFinder(const std::string& rName)
  : m_Name(rName)
  , m_NumberOfRangeReadings(0)
  {
    m_pMinimumRange = 0.0;
    m_pMaximumRange = 80.0;

    m_pMinimumAngle = -KT_PI_2;
    m_pMaximumAngle = KT_PI_2;

    m_pAngularResolution = math::DegreesToRadians(1);

    m_pRangeThreshold = 12.0;
  }

  /**
   * Set the number of range readings based on the minimum and
   * maximum angles of the sensor and the angular resolution
   */
  void Update()
  {
    m_NumberOfRangeReadings = static_cast<kt_int32u>(math::Round((GetMaximumAngle() -
                                  GetMinimumAngle()) / 
                                  GetAngularResolution()) + 1);
  }

private:
  LaserRangeFinder(const LaserRangeFinder&);
  const LaserRangeFinder& operator=(const LaserRangeFinder&);

private:
  // sensor m_Parameters
  kt_double m_pMinimumAngle;
  kt_double m_pMaximumAngle;

  kt_double m_pAngularResolution;

  kt_double m_pMinimumRange;
  kt_double m_pMaximumRange;

  kt_double m_pRangeThreshold;

  kt_int32u m_NumberOfRangeReadings;

  /**
  * Sensor offset pose
  */
  Pose2 m_pOffsetPose;

  std::string m_Name;
};  // LaserRangeFinder


/**
 * The LocalizedRangeScan contains range data from a single sweep of a laser range finder sensor
 * in a two-dimensional space and position information. The odometer position is the position
 * reported by the robot when the range data was recorded. The corrected position is the position
 * calculated by the mapper (or localizer)
 */
class LocalizedRangeScan : public LaserRangeScan
{
public:
  /**
   * Constructs a range scan from the given range finder with the given readings
   */
  LocalizedRangeScan(LaserRangeFinder* rLaserRangeFinder, const RangeReadingsVector& rReadings)
  : LaserRangeScan(rLaserRangeFinder->GetName(), rReadings)
  , m_LaserRangeFinder(rLaserRangeFinder)
  , m_IsDirty(true)
  {
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScan()
  {
  }

private:
  mutable boost::shared_mutex m_Lock;

public:
  /**
   * Gets the odometric pose of this scan
   * @return odometric pose of this scan
   */
  inline const Pose2& GetOdometricPose() const
  {
    return m_OdometricPose;
  }

  /**
   * Sets the odometric pose of this scan
   * @param rPose
   */
  inline void SetOdometricPose(const Pose2& rPose)
  {
    m_OdometricPose = rPose;
  }

  /**
   * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
   * is usually set by an external module such as a localization or mapping module when it is determined
   * that the original pose was incorrect.  The external module will set the correct pose based on
   * additional sensor data and any context information it has.  If the pose has not been corrected,
   * a call to this method returns the same pose as GetOdometricPose().
   * @return corrected pose
   */
  inline const Pose2& GetCorrectedPose() const
  {
    return m_CorrectedPose;
  }

  /**
   * Moves the scan by moving the robot pose to the given location.
   * @param rPose new pose of the robot of this scan
   */
  inline void SetCorrectedPose(const Pose2& rPose)
  {
    m_CorrectedPose = rPose;

    m_IsDirty = true;
  }

  /**
   * Gets barycenter of point readings
   */
  inline const Pose2& GetBarycenterPose() const
  {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty)
    {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    return m_BarycenterPose;
  }

  /**
   * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
   * @param useBarycenter
   * @return barycenter if parameter is true, otherwise scanner pose
   */
  inline Pose2 GetReferencePose(kt_bool useBarycenter) const
  {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty)
    {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    return useBarycenter ? GetBarycenterPose() : GetSensorPose();
  }

  /**
   * Computes the position of the sensor
   * @return scan pose
   */
  inline Pose2 GetSensorPose() const
  {
    return GetSensorAt(m_CorrectedPose);
  }

  /**
   * Computes the robot pose given the corrected scan pose
   * @param rScanPose pose of the sensor
   */
  void SetSensorPose(const Pose2& rScanPose)
  {
    Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
    kt_double offsetLength = deviceOffsetPose2.GetPosition().Length();
    kt_double offsetHeading = deviceOffsetPose2.GetHeading();
    kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
    kt_double correctedHeading = math::NormalizeAngle(rScanPose.GetHeading());
    Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
                    offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
                    offsetHeading);

    m_CorrectedPose = rScanPose - worldSensorOffset;

    Update();
  }

  /**
   * Computes the position of the sensor if the robot were at the given pose
   * @param rPose
   * @return sensor pose
   */
  inline Pose2 GetSensorAt(const Pose2& rPose) const
  {
    return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
  }

  /**
   * Gets the bounding box of this scan
   * @return bounding box of this scan
   */
  inline const BoundingBox2& GetBoundingBox() const
  {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty)
    {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    return m_BoundingBox;
  }

  /**
   * Get point readings in local coordinates
   */
  inline const PointVectorDouble& GetPointReadings(kt_bool wantFiltered = false) const
  {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty)
    {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    if (wantFiltered == true)
    {
      return m_PointReadings;
    }
    else
    {
      return m_UnfilteredPointReadings;
    }
  }

  /**
   * Gets the laser range finder sensor that generated this scan
   * @return laser range finder sensor of this scan
   */
  inline LaserRangeFinder* GetLaserRangeFinder() const
  {
    return m_LaserRangeFinder;
  }

private:
  /**
   * Compute point readings based on range readings
   * Only range readings within [minimum range; range threshold] are returned
   */
  virtual void Update()
  {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

    if (pLaserRangeFinder != NULL)
    {
      m_PointReadings.clear();
      m_UnfilteredPointReadings.clear();

      kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetSensorPose();

      // compute point readings
      Vector2<kt_double> rangePointsSum;
      kt_int32u beamNum = 0;
      for (kt_int32u i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++)
      {
        kt_double rangeReading = GetRangeReadings()[i];
        if (!math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
        {
          kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

          Vector2<kt_double> point;
          point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
          point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

          m_UnfilteredPointReadings.push_back(point);
          continue;
        }

        kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Vector2<kt_double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        m_PointReadings.push_back(point);
        m_UnfilteredPointReadings.push_back(point);

        rangePointsSum += point;
      }

      // compute barycenter
      kt_double nPoints = static_cast<kt_double>(m_PointReadings.size());
      if (nPoints != 0.0)
      {
        Vector2<kt_double> averagePosition = Vector2<kt_double>(rangePointsSum / nPoints);
        m_BarycenterPose = Pose2(averagePosition, 0.0);
      }
      else
      {
        m_BarycenterPose = scanPose;
      }

      // calculate bounding box of scan
      m_BoundingBox = BoundingBox2();
      m_BoundingBox.Add(scanPose.GetPosition());
      forEach(PointVectorDouble, &m_PointReadings)
      {
        m_BoundingBox.Add(*iter);
      }
    }

    m_IsDirty = false;
  }

private:
  LocalizedRangeScan(const LocalizedRangeScan&);
  const LocalizedRangeScan& operator=(const LocalizedRangeScan&);

private:
  /**
   * Odometric pose of robot
   */
  Pose2 m_OdometricPose;

  /**
   * Corrected pose of robot calculated by mapper (or localizer)
   */
  Pose2 m_CorrectedPose;

protected:
  /**
   * Average of all the point readings
   */
  Pose2 m_BarycenterPose;

  /**
   * Vector of point readings
   */
  PointVectorDouble m_PointReadings;

  /**
   * Vector of unfiltered point readings
   */
  PointVectorDouble m_UnfilteredPointReadings;

  /**
   * Bounding box of localized range scan
   */
  BoundingBox2 m_BoundingBox;

  /**
   * Internal flag used to update point readings, barycenter and bounding box
   */
  kt_bool m_IsDirty;

  LaserRangeFinder* m_LaserRangeFinder;

};  // LocalizedRangeScan

/**
 * Type declaration of LocalizedRangeScan vector
 */
typedef std::vector<LocalizedRangeScan*> LocalizedRangeScanVector;

}  // namespace KartoScanMatcher

#endif  // _LASER_SENSOR_H_