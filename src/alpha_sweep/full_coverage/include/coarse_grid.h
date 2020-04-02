/*-------------------------------------------------------------------------------
 coarse_grid.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Stores and calculates coarse_grid data for the full coverage planner.
-------------------------------------------------------------------------------*/

#ifndef _COARSE_GRID_H_
#define _COARSE_GRID_H_

// ROS header for ROS functionality
#include "ros/ros.h"

#include "coverage_grid.h"
#include "coarse_cell.h"

#include "full_coverage/CellGrid.h"

namespace full_coverage
{
  
/**
* CoarseGrid construct
*/
class CoarseGrid : public CoverageGrid {
private:
  std::vector<CoarseCell> coarseCells;         /*!< Vector of coarse cells */
  CellGrid map;                                /*!< Occupancy map */
  int occupancyThreshold;                      /*!< Threshold to call a cell occupied */
  int numBorderCells;                          /*!< Number of cells from each obstacle or unknown cell to classify as a border cell */

  /**
   * @brief Initialize the cells contained in the coarse grid.
   */
  bool initializeCells();

  /**
   * @brief Find the neighbors of each cell and set the neighbor vectors of each cell.
   */
  void findNeighbors();

  /**
   * @brief Sets all cells within numBorderCells of an occupied or unknown cell as a border cell.
   */
  void setBorders();
  
public:
  /**
   * @brief Constructor.
   */
  CoarseGrid();

  /**
   * @brief Destructor.
   */
  ~CoarseGrid();

  /**
    * @brief implicitly defined copy constructor
    */
  CoarseGrid(const CoarseGrid &cg);

  /**
   * @brief Initializes the coarse grid.
   * @param mSize  Size of the grid (max index + 1)
   * @param nCol   Number of columns in grid
   * @return True if initialization was successful
   */
  bool initialize(CellGrid& m, int ot, int nbc);
   
  /**
   * @brief Set all neighbors of this cell to the waWvefront value
   * @param index Index of cell who's neighbors are to be set
   * @param val  Wavefront value to set neighbors to
   * @return Number of neighbors that were set to the passed cell value
   */
  int setNeighbors(int index, CellValue val);
  
}; // CoarseGrid

} // namespace full_coverage

#endif // _COARSE_GRID_H_
