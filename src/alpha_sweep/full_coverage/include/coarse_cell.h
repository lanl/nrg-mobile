/*-------------------------------------------------------------------------------
 coarse_cell.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Holds and calculates coarse cell data for one cell of the map.
-------------------------------------------------------------------------------*/

#ifndef _COARSE_CELL_H_
#define _COARSE_CELL_H_

// Base class
#include "coverage_cell.h"

// ROS infrastructure
#include "ros/ros.h"

// Custom messages
#include "full_coverage/SquareCell.h"

namespace full_coverage
{
  
/**
* CoarseCell construct holds coarse cell data for a single cell
*/
class CoarseCell : public CoverageCell {
private:
  SquareCell mapCell;
  int occupancyProb;                      /*!< The occupancy of the cell */
  int occupancyThreshold;                 /*!< The percentage of occupancy that qualifies the cell to be obstructed */
  
  void setCorners();

public:
  /**
   * @brief Constructor.
   */
  CoarseCell();

  /**
   * @brief Destructor.
   */
  ~CoarseCell();

  /**
   * @brief Initialize method.
   * @param pos Index of this cell in global map
   * @param mc  Map data corresponding to this cell
   * @param ot  Occupancy threshold for the map grid
   */
  void initialize(int pos, SquareCell& mc, int ot);
  
  /**
   * @brief Set all neighbors of this cell to the cell value
   * @param value Cell value to set neighbors to
   * @return Number of neighbors that were set to the passed cell value
   */
  int setNeighbors(CellValue val);
  
}; // CoarseCell

} // namespace full_coverage

#endif // _COARSE_CELL_H_
