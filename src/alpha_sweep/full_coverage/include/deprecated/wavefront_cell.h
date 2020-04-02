/*-------------------------------------------------------------------------------
 wavefront_cell.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Holds and calculates wavefront data for one cell of the map.
-------------------------------------------------------------------------------*/

#ifndef _WAVEFRONT_CELL_H_
#define _WAVEFRONT_CELL_H_

// Base class
#include "coverage_cell.h"

// ROS infrastructure
#include "ros/ros.h"

// Custom messages
#include "full_coverage/SquareCell.h"

namespace full_coverage
{
  
/**
* WavefrontCell construct holds wavefront data for a single cell
*/
class WavefrontCell : public CoverageCell {
private:
  SquareCell mapCell;
  int occupancyProb;                      /*!< The occupancy of the cell */
  int occupancyThreshold;                 /*!< The percentage of occupancy that qualifies the cell to be obstructed */
  
  void setCorners();

public:
  /**
   * @brief Constructor.
   */
  WavefrontCell();

  /**
   * @brief Destructor.
   */
  ~WavefrontCell();

  /**
   * @brief Initialize method.
   * @param pos Index of this cell in global map
   * @param mc  Map data corresponding to this cell
   * @param ot  Occupancy threshold for the map grid
   */
  void initialize(int pos, SquareCell& mc, int ot);
  
  /**
   * @brief Set the wavefront value of this cell
   * @param value The wavefront value
   * @return True if the cell was set
   */
  bool setValue(int val);

  /**
   * @brief Set all neighbors of this cell to the wavefront value
   * @param wave Wavefront value to set neighbors to
   * @return Number of neighbors that were set to the passed wave number
   */
  int setNeighbors(int wave);
  
  /**
   * @brief Reset the cell wave value and isPlanned variable
   */
  void reset();

}; // WavefrontCell

} // namespace full_coverage

#endif // _WAVEFRONT_CELL_H_
