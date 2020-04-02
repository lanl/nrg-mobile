/*-------------------------------------------------------------------------------
 coverage_grid.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Stores and calculates coverage_grid data for the full coverage planner.
-------------------------------------------------------------------------------*/

#ifndef _COVERAGE_GRID_H_
#define _COVERAGE_GRID_H_

// ROS header for ROS functionality
#include "ros/ros.h"

#include "coverage_cell.h"

// open cv for image printing
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace full_coverage
{
  enum MapDirection
  {
    MAP_NORTH = 1,
    MAP_EAST  = 2,
    MAP_SOUTH = 3,
    MAP_WEST  = 4
  };
  
/**
* CoverageGrid construct
*/
class CoverageGrid {
private:
  int mapSize;                             /*!< Number of cells in the grid */
  int numRows;                             /*!< Number of cell rows in the grid */
  int numCols;                             /*!< Number of cell columns in the grid */
  bool gridInit;
protected:  
    /**
   * @brief Adds neighbor at position (r,c) to the neighbors vector if it is valid.
   * @param neighbors Vector of neighbors
   * @param r         Row of proposed neighbor
   * @param c         Column of proposed neighbor
   */
  virtual void addNeighbor(std::vector<CoverageCell*> &neighbors, int r, int c);
  
    /**
   * @brief Finds and sets valid neighbors of all coverage_grid cells. Occupied cells are invalid neighbors.
   */
  virtual void findNeighbors();

  virtual bool initializeCells();

  double getDistance(geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2);

  std::vector<CoverageCell*> cells;        /*!< Vector coverage cell pointers */
  
  bool isInitialized;                      /*!< True if class was initialized successfully */

public:
  /**
   * @brief Constructor.
   */
  CoverageGrid();

  /**
   * @brief Destructor.
   */
  ~CoverageGrid();

  /**
    * @brief Implicitly defined copy constructor
    * */
  CoverageGrid(const CoverageGrid &cg);
  
  /**
   * @brief Initializes the coverage_grid.
   * @param mSize  Size of the grid (max index + 1)
   * @param nCol   Number of columns in grid
   * @return True if initialization was successful
   */
  virtual bool initialize(int mSize, int nCol);
  
  /**
   * @brief Transforms index into (r,c) getCoordinate if the index is valid
   * @param index Index of the cell
   * @param r     Row number corresponding to the given index
   * @param c     Column number corresponding to the given index
   * @return      True if the index was valid
   */
  bool getCoordinate(int index, int &r, int &c);
  
  /**
   * @brief Transforms (r,c) getCoordinate into a cell index if the coordinate is valid
   * @param index Index of the cell at (r,c)
   * @param r     Row number of the cell
   * @param c     Column number of the cell
   * @return      True if the (r,c) coordinate was valid
   */
  bool getIndex(int &index, int r, int c);
  
  /**
   * @brief Determines which cell contains the given pose
   * @param pose The pose we want the cell index of
   * @return Index of the cell containing the given pose
   */
  virtual int getIndex(geometry_msgs::Pose pose);
  
   /**
   * @brief Finds and returns the next closest valid and unplanned cell to the cell at the passed index.
   * @param index The index of the cell that the wavefront algorithm got stuck at and needs to be changed.
   * @return True if we hopped succesfully
   */
  bool hop(int &index);
  bool findReachable(int index);

   /**
   * @brief Get the neighbors vector of a cell in the grid
   * @param index Index of the cell whose neighbors are retreived
   * @param nbs Vector of pointers to neighboring cells
   * @return True if we got the neighbors
   */
  bool getNeighbors(int index, std::vector<CoverageCell*>& nbs);
    
  /**
   * @brief Get unplanned neighbors of a cell in the grid
   * @param index Index of the cell whose unplanned neighbors are retreived
   * @param unbs Vector of pointers to unplanned neighbors of the cell
   * @return True if we got the unplanned neighbors
   */
  bool getUnplannedNeighbors(int index, std::vector<CoverageCell*>& unbs);
  
  /**
   * @brief Get the wavefront value of a cell in the grid
   * @param index Index of the cell whose value is retreived
   * @param val   The wavefront value of the cell
   * @return True if we got the value
   */
  bool getValue(int index, CellValue &val);
  
  /**
   * @brief Set the integer value of a cell in the grid
   * @param index Index of the cell whose value is to be changed
   * @param val   Wavefront value to set
   * @return True if the value was set
   */
  bool setValue(int index, CellValue val);
  
   /**
   * @brief Plan a cell in the grid
   * @param index Index of the cell to be planned
   * @return True if we could plan the cell
   */
  virtual bool setPlanned(int index);

  /**
  * @brief Visit a cell in the grid
  * @param index Index of the cell to be visited
  * @return True if we could visit the cell
  */
  virtual bool visit(int index);
  virtual bool visit(geometry_msgs::Pose pose);

  virtual bool setAttempted(int index);
  virtual bool setAttempted(geometry_msgs::Pose pose);

  bool setReachable(int index, bool val);
  bool isReachable(int index, bool &val);

  /**
   * @brief Check if a cell in the grid has been planned
   * @param index Index of the cell to check
   * @param val   True if the cell has been planend
   * @return True if we could check if the cell has been planned
   */
  bool isPlanned(int index, bool &val);

  /**
   * @brief Check if a cell in the grid has been visited by the robot
   * @param index Index of the cell to check
   * @param val   True if the cell has been visited
   * @return True if we could check if the cell has been visited
   */
  bool isVisited(int index, bool &val);

  bool isAttempted(int index, bool &val);

  void getUnvisitedCells(std::vector<geometry_msgs::Pose> &poses);
  void getUnvisitedCells(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds);

  /**
   * @brief Check if the pose is contained in this cell's boundaries
   * @param index Index of the cell to check
   * @param pose  The pose to check
   * @param val   True if the pose is contained within this cell
   * @return True if we were able to check the cell at index
   */
  bool contains(int index, geometry_msgs::Pose pose, bool &val);
  
  /**
   * @brief Get the center pose of this cell
   * @param index Index of the cell whose pose is to be retreived
   * @param pose  The pose in the center of this cell. Orientation will be left to default value.
   * @return True if the pose was retreived
   */
  bool getPose(int index, geometry_msgs::Pose &pose);  
  virtual void getPositions(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds);
  /**
   * @brief Gets the pose of each corner
   * @param index Index of the cell whose corners are to be retreived
   * @param poses Array of corner poses. Should be in order: tl, tr, bl, br.
   * @return True if we could get corners of cell at index.
   **/
   bool getCorners(int index, std::vector<geometry_msgs::Pose> &poses);
  
   /**
   * @brief Sets the pose of each corner
   * @param index Index of the cell whose corners are to be set
   * @param verts Array of corner poses. Should be in order: tl, tr, bl, br.
   * @return True if corners were set
   **/
  bool setCorners(int index, std::vector<geometry_msgs::Pose> verts);
  
  void print(std::string name);

  int getSize();
  int getNumCols();
  int getNumRows();
  
  bool getCell(int index, CoverageCell* &cell);

  double getCellWidth();
}; // CoverageGrid

} // namespace full_coverage

#endif // _COVERAGE_GRID_H_
