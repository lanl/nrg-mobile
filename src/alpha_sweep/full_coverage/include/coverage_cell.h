/*-------------------------------------------------------------------------------
 coverage_cell.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Holds and calculates wavefront data for one cell of the map.
-------------------------------------------------------------------------------*/

#ifndef _COVERAGE_CELL_H_
#define _COVERAGE_CELL_H_

// ROS infrastructure
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cell_bounds.h"

namespace full_coverage
{

enum CellValue
{
  OPEN_CELL        = 0,        /*!< Value used for end cell (must be zero because wave starts propogating upward from here) */
  BORDER_CELL      = 1,        /*!< Value used for a cell who borders an obstruction */
  OBSTRUCTED_CELL  = 2,        /*!< Value used for obstruction */
  UNKNOWN_CELL     = 3         /*!< Value used fot initialization */
};

/**
* CoverageCell construct holds wavefront data for a single cell
*/
class CoverageCell {
private:
  int position;                            /*!< Index of this cell in global map */
  CellValue value;                         /*!< Value of this coverage cell defined by CellValue enum */
  bool cellPlanned;                        /*!< True if this cell has been planned by the planner */
  bool cellVisited;                        /*!< True if this cell has been visited by the robot */
  bool cellAttempted;                      /*!< True if the robot has attempted to visit the cell */
  bool cellReachable;                      /*!< True if the coverage algorithm can reach this cell from the robots current position. */
  CellBounds bounds;                       /*!< Coordinate bounds of the cell */
  
  /**
   * @brief Virtual method to be implemented by derived classes to set the corners of the cell.
   */
  virtual void setCorners() = 0;

protected:
  bool isInitialized;                      /*!< True if this cell was initialized correctly */
  std::vector<CoverageCell*> neighbors;    /*!< Vector containing pointers to valid neighbors of this cell */
  
public:
  /**
   * @brief Constructor.
   */
  CoverageCell();

  /**
   * @brief Destructor.
   */
  ~CoverageCell();

  /**
   * @brief Initialize method.
   * @param pos Index of this cell in global map
   */
  void initialize(int pos);
  
  /**
   * @brief Adds a neighbor to the neighbors vector
   * @param nb Pointers to a neighbor of this cell
   */
  void addNeighbor(CoverageCell* nb);
  
  /**
   * @brief Clear neighbors vector.
   */
  void clearNeighbors();
  
  /**
   * @brief Get the neighbors vector
   * @param nbs Vector of pointers to neighboring cells
   */
  void getNeighbors(std::vector<CoverageCell*>& nbs);
  
  /**
   * @brief Get unplanned neighbors of this cell
   * @param unbs Vector of pointers to unplanned neighbors of this cell
   */
  void getUnplannedNeighbors(std::vector<CoverageCell*>& unbs);
  
  /**
   * @brief Get the wavefront value of the cell
   * @return The wavefront value of the cell
   */
  virtual CellValue getValue();

  /**
   * @brief Set the integer value of the cell
   * @return The integer value of the cell
   */
  virtual bool setValue(CellValue val);
  
  /**
   * @brief Get the index of this cell
   * @return Index of this cell
   */
  int getPosition();

  /**
   * @brief Set the cell as planned
   */
  virtual void setPlanned();

  /**
   * @brief Set the cell as visited
   */
  virtual void visit();

  /**
   * @brief Set the cell as visited
   */
  virtual void setAttempted();

  /**
   * @brief Check if the cell has been planned
   * @return True if the cell has been planend
   */
  virtual bool isPlanned();

  /**
   * @brief Check if some percent of the cell has been planned
   * @param percent The percent of the cell that is planned to return true.
   * @return True if the cell has been planend
   */
  virtual bool isPlanned(double percent);

  /**
   * @brief Check if the cell has been visited by the robot
   * @return True if the cell has been visited by the robot
   */
  virtual bool isVisited();

  /**
   * @brief Check if the some percent of the cell has been visited by the robot
   * @param percent The percent of the cell that is visited to return true
   * @return True if the cell has been visited by the robot
   */
  virtual bool isVisited(double percent);

  /**
   * @brief Check if the cell has been visited by the robot
   * @return True if the cell has been visited by the robot
   */
  virtual bool isAttempted();

  /**
   * @brief Check if the robot has attempted to visit some percent of the cell
   * @param percent The percent of the cell that is attempted to return true
   * @return True if the cell has been visited by the robot
   */
  virtual bool isAttempted(double percent);

  /**
   * @brief Check if the cell is reachable by the robot
   * @return True if the cell is reachable by the robot
   */
  bool isReachable();

  /**
   * @brief Sets the cells reachable value
   * @param ir Pass true if the cell is reachable, otherwise false
   */
  void setReachable(bool ir);

  /**
   * @brief Check if the pose is contained in this cell's boundaries
   * @param pose The pose to check
   * @return True if the pose is contained within this cell
   */
  bool contains(geometry_msgs::Pose pose);
  
  /**
   * @brief Get the center pose of this cell
   * @retrun The pose in the center of this cell. Orientation will be left to default value.
   */
  geometry_msgs::Pose getPose();  

  /**
   * @brief Gets the pose of each corner
   * @return Array of corner poses. Should be in order: tl, tr, bl, br.
   **/
  std::vector<geometry_msgs::Pose> getCorners();
  
   /**
   * @brief Sets the pose of each corner
   * @param inh Array of poses of cell inhabitants.
   **/
  void setCorners(std::vector<geometry_msgs::Pose> allCorners);

  /**
   * @brief Gets the width of the cell
   * @return The width of the cell.
   **/
  double getWidth();
  
}; // CoverageCell

} // namespace full_coverage

#endif // _COVERAGE_CELL_H_
