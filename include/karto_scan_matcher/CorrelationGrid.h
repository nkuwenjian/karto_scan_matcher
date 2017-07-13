#ifndef _CORRELATION_GRID_H_
#define _CORRELATION_GRID_H_

#include <karto_scan_matcher/Grid.h>
#include <karto_scan_matcher/DataStructure.h>

#include <stdexcept>

namespace KartoScanMatcher
{
/**
 * Implementation of a correlation grid used for scan matching
 */
class CorrelationGrid : public Grid<kt_int8u>
{
public:
	/**
	 * Destructor
	 */
	virtual ~CorrelationGrid()
	{
		delete [] m_pKernel;
	}

public:
	/**
	 * Create a correlation grid of given size and parameters
	 * @param width
	 * @param height
	 * @param resolution
	 * @param smearDeviation
	 * @return correlation grid
	 */
	static CorrelationGrid* CreateGrid(kt_int32s width,
										kt_int32s height,
										kt_double resolution,
										kt_double smearDeviation)
	{
		assert(resolution != 0.0);

		// +1 in case of roundoff
		kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;  // borderSize = 7

		CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);

		return pGrid;
	}

	/**
	 * Gets the index into the data pointer of the given grid coordinate
	 * @param rGrid
	 * @param boundaryCheck
	 * @return grid index
	 */
	virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = true) const
	{
		kt_int32s x = rGrid.GetX() + m_Roi.GetX();
		kt_int32s y = rGrid.GetY() + m_Roi.GetY();

		return Grid<kt_int8u>::GridIndex(Vector2<kt_int32s>(x, y), boundaryCheck);
	}

	/**
	 * Get the Region Of Interest (ROI)
	 * @return region of interest
	 */
	inline const Rectangle2<kt_int32s>& GetROI() const
	{
		return m_Roi;
	}

	/**
	 * Sets the Region Of Interest (ROI)
	 * @param roi
	 */
	inline void SetROI(const Rectangle2<kt_int32s>& roi)
	{
		m_Roi = roi;
	}

	/**
	 * Smear cell if the cell at the given point is marked as "occupied"
	 * @param rGridPoint
	 */
	inline void SmearPoint(const Vector2<kt_int32s>& rGridPoint)
	{
		assert(m_pKernel != NULL);

		int gridIndex = GridIndex(rGridPoint);
		if (GetDataPointer()[gridIndex] != GridStates_Occupied)
		{
			return;
		}

		kt_int32s halfKernel = m_KernelSize / 2;  // m_KernelSize = 13, halfKernel = 6

		// apply kernel
		for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
		{
			kt_int8u* pGridAdr = GetDataPointer(Vector2<kt_int32s>(rGridPoint.GetX(), rGridPoint.GetY() + j));

			kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);

			// if a point is on the edge of the grid, there is no problem
			// with running over the edge of allowable memory, because
			// the grid has margins to compensate for the kernel size
			for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
			{
				kt_int32s kernelArrayIndex = i + kernelConstant;

				kt_int8u kernelValue = m_pKernel[kernelArrayIndex];
				if (kernelValue > pGridAdr[i])
				{
					// kernel value is greater, so set it to kernel value
					pGridAdr[i] = kernelValue;
				}
			}
		}
	}

protected:
	/**
	 * Constructs a correlation grid of given size and parameters
	 * @param width
	 * @param height
	 * @param borderSize
	 * @param resolution
	 * @param smearDeviation
	 */
	CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize,
					kt_double resolution, kt_double smearDeviation)
		: Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
		, m_SmearDeviation(smearDeviation)
		, m_pKernel(NULL)
	{
		GetCoordinateConverter()->SetScale(1.0 / resolution);

		// setup region of interest
		m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);

		// calculate kernel
		CalculateKernel();
	}

	/**
	 * Sets up the kernel for grid smearing.
	 */
	virtual void CalculateKernel()
	{
		kt_double resolution = GetResolution();

		assert(resolution != 0.0);
		assert(m_SmearDeviation != 0.0);

		// min and max distance deviation for smearing;
		// will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
		const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
		const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

		// check if given value too small or too big
		if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
		{
			std::stringstream error;
			error << "Mapper Error:  Smear deviation too small:  Must be between "
					<< MIN_SMEAR_DISTANCE_DEVIATION
					<< " and "
					<< MAX_SMEAR_DISTANCE_DEVIATION;
			throw std::runtime_error(error.str());
		}

		// NOTE:  Currently assumes a two-dimensional kernel

		// +1 for center
		m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;  // m_KernelSize = 13

		// allocate kernel
		m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
		if (m_pKernel == NULL)
		{
			throw std::runtime_error("Unable to allocate memory for kernel!");
		}

		// calculate kernel
		kt_int32s halfKernel = m_KernelSize / 2;  // halfKernel = 6
		for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
		{
			for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
			{
#ifdef WIN32
				kt_double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
				kt_double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
				kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

				kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
				assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));

				int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
				m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
			}
		}
	}

	/**
	 * Computes the kernel half-size based on the smear distance and the grid resolution.
	 * Computes to two standard deviations to get 95% region and to reduce aliasing.
	 * @param smearDeviation
	 * @param resolution
	 * @return kernel half-size based on the parameters
	 */
	static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
	{
		assert(resolution != 0.0);

		return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
	}

private:
	/**
	 * The point readings are smeared by this value in X and Y to create a smoother response.
	 * Default value is 0.03 meters.
	 */
	kt_double m_SmearDeviation;

	// Size of one side of the kernel
	kt_int32s m_KernelSize;

	// Cached kernel for smearing
	kt_int8u* m_pKernel;

	// region of interest
	Rectangle2<kt_int32s> m_Roi;
};  // CorrelationGrid

}  // namespace KartoScanMatcher

#endif  // _CORRELATION_GRID_H_