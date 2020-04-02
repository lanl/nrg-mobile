/*-------------------------------------------------------------------------------
 robot_cell.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Holds and calculates wavefront data for one cell of the map.
-------------------------------------------------------------------------------*/

#ifndef _ROBOT_CELL_H_
#define _ROBOT_CELL_H_

// Super class
#include "coverage_cell.h"

// ROS infrastructure
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

// Contained cells
#include "coarse_cell.h"

namespace full_coverage
{
  
/**
* RobotCell construct contains multiple coarse cells
*/
class RobotCell : public CoverageCell {
private:
  std::vector<CoverageCell*> inhabitants; /*!< Vector containing pointers to wavefront cells contained within this robot cell */
  void setCorners();

public:
  /**
   * @brief Constructor.
   */
  RobotCell();

  /**
   * @brief Destructor.
   */
  ~RobotCell();

  /**
   * @brief Initialize method.
   * @param pos Index of this cell in global map
   * @param inh Array of pointers to wavefront cells contained in this robot cell.
   */
  void initialize(int pos, std::vector<CoverageCell*> inh);

  CellValue updateValue();
  
  /**
   * @brief Set all contained wavefront cells as planned
   * @param numChanged Number of cells planned that were previously unplanned
   */
  void setPlanned(int &numChanged);

  /**
   * @brief Set all contained wavefront cells as visited
   */
  void visit();

  bool isPlanned();
  bool isPlanned(int index);
  bool isPlanned(double percent);
  bool isVisited();
  bool isVisited(int index);
  bool isVisited(double percent);
  bool isAttempted();
  bool isAttempted(int index);
  bool isAttempted(double percent);
  void updatePlanned();
  void updateVisited();
  void updateAttempted();
  bool setValue(CellValue val);
  /**
   * @brief Check what number of contained wavefront cells have been planned
   * @return Number of contained wavefront cells that have been planend
   */
  int numPlanned();
  bool getCoarseIndex(int &ind);

}; // RobotCell

} // namespace full_coverage

#endif // _ROBOT_CELL_H_
