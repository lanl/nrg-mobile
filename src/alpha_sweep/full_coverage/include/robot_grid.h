/*-------------------------------------------------------------------------------
 robot_grid.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Stores and calculates robot_grid data for the full coverage planner.
-------------------------------------------------------------------------------*/

#ifndef _ROBOT_GRID_H_
#define _ROBOT_GRID_H_

// ROS header for ROS functionality
#include "ros/ros.h"

#include "coverage_grid.h"
#include "coarse_grid.h"
#include "robot_cell.h"

#include "full_coverage/CellGrid.h"

namespace full_coverage
{
  
/**
* RobotGrid construct
*/
class RobotGrid : public CoverageGrid {
private:
  std::vector<RobotCell> robotCells;           /*!< Vector of robot cells */
  CoarseGrid* coarseGrid;
  int cellDim;
  bool secondAttempt;
  bool initializeCells();
  bool isPlannedLeft(int r, int c, MapDirection currentDir);
  bool isPlannedAhead(int r, int c, MapDirection currentDir);
  bool isWallLeft(int r, int c, MapDirection currentDir);
  bool isWallAhead(int r, int c, MapDirection currentDir);  
  bool isOpenAhead(int r, int c, MapDirection currentDir);
  void getLeftInc(int &ri, int &ci, MapDirection dir);
  void getAheadInc(int &ri, int &ci, MapDirection dir);
  bool isWall(int r, int c, int dist, MapDirection dir);
  MapDirection getLeft(MapDirection dir);
  MapDirection getRight(MapDirection dir);
  void getSurroundingCells(std::vector<int> &ind, bool &foundNonObs, bool allowUnknown = false);
public:
  /**
   * @brief Constructor.
   */
  RobotGrid();

  /**
   * @brief Destructor.
   */
  ~RobotGrid();

  /**
  * @brief Implicitly defined copy constructor
  */
  RobotGrid(const RobotGrid &rg, CoarseGrid *cg = 0);

  /**
   * @brief Initializes the robot_grid.
   * @param mSize  Size of the grid (max index + 1)
   * @param nCol   Number of columns in grid
   * @return True if initialization was successful
   */
  bool initialize(CoarseGrid* wg, int cd);
  
  /**
   * @brief Finds and sets valid neighbors of all coverage_grid cells. Occupied cells are invalid neighbors.
   */
  void findNeighbors();

  bool findOpenNeighbor(int &index, double openFraction = 0.0, bool allowUnknown = false);

  bool updateValue(int index);
  
  bool getBorderNeighbor(int index, MapDirection dir, int &val, MapDirection &newDir);

  MapDirection getWallDirection(int index);
  
  bool leaveHall(int index, MapDirection dir, int &val, MapDirection &newDir);
  
  bool findWall(int index, MapDirection dir, int& val, MapDirection &newDir, bool &found);
  /**
   * @brief Plan a cell in the grid
   * @param index      Index of the cell to be planned
   * @param numChanged Number of previously unplanned cells that are planned
   * @return True if we could set the cell as planned
   */
  bool setPlanned(int index, int &numChanged);

  /**
   * @brief Visit a cell in the grid
   * @param index      Index of the cell to be visited
   * @return True if we could set the cell as visited
   */
  //bool visit(int index);
  //bool visit(geometry_msgs::Pose pose);

  //using CoverageGrid::getIndex;
  //int getIndex(geometry_msgs::Pose pose);
  void setSecondAttempt(bool sec);
  void getPositions(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds);
  bool getCoarseIndex(int &ind);

  int getWidth();
  int getHeight();
  
}; // RobotGrid

} // namespace full_coverage

#endif // _ROBOT_GRID_H_
