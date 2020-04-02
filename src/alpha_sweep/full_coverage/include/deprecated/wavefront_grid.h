/*-------------------------------------------------------------------------------
 wavefront_grid.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Stores and calculates wavefront_grid data for the full coverage planner.
-------------------------------------------------------------------------------*/

#ifndef _WAVEFRONT_GRID_H_
#define _WAVEFRONT_GRID_H_

// ROS header for ROS functionality
#include "ros/ros.h"

#include "coverage_grid.h"
#include "wavefront_cell.h"

#include "full_coverage/CellGrid.h"

namespace full_coverage
{
  
/**
* WavefrontGrid construct
*/
class WavefrontGrid : public CoverageGrid {
private:
  std::vector<WavefrontCell> waveCells;        /*!< Vector of wavefront cells */
  CellGrid map;                                /*!< Occupancy map */
  int occupancyThreshold;                      /*!< Threshold to call a cell occupied */
  int numBorderCells;
  
  bool initializeCells();
  
  void findNeighbors();
  
  void setBorders();
  
public:
  /**
   * @brief Constructor.
   */
  WavefrontGrid();

  /**
   * @brief Destructor.
   */
  ~WavefrontGrid();

  /**
   * @brief Initializes the wavefront_grid.
   * @param mSize  Size of the grid (max index + 1)
   * @param nCol   Number of columns in grid
   * @return True if initialization was successful
   */
  bool initialize(CellGrid& m, int ot, int nbc);
   
  /**
   * @brief Set all neighbors of this cell to the waWvefront value
   * @param index Index of cell who's neighbors are to be set
   * @param wave  Wavefront value to set neighbors to
   * @return Number of neighbors that were set to the passed wave number
   */
  int setNeighbors(int index, int wave);
  
  void print();
  
}; // WavefrontGrid

} // namespace full_coverage

#endif // _WAVEFRONT_GRID_H_
