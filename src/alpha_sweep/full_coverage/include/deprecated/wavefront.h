/*-------------------------------------------------------------------------------
 wavefront.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Stores and calculates wavefront data for the wavefront planner.
-------------------------------------------------------------------------------*/

#ifndef _WAVEFRONT_H_
#define _WAVEFRONT_H_

// ROS header for ROS functionality
#include "ros/ros.h"

// Wavefront and robot grid constructs
#include "wavefront_grid.h"
#include "robot_grid.h"

// Map data construct
#include "full_coverage/CellGrid.h"

// TF (ROS package for transforming poses)
#include "tf/transform_datatypes.h"

namespace full_coverage
{
  
/**
* Wavefront construct
*/
class Wavefront {
private:
  WavefrontGrid waveCells;         /*!< Vector of wave cells that store wavefront data for each cell */
  RobotGrid robotCells;            /*!< Vector of robot cells that contain wavefront cells whose area equals the robot area */
  int startIndex;                  /*!< Index of the cell that the plan should start at (current position of the robot) */
  int endIndex;                    /*!< Index of the cell that the plan will end at (adjacent to start cell) */
  bool isInitialized;              /*!< True if class was initialized successfully */
  
    /**
   * @brief Adds the pose of the cell at the passed index to the plan accounting for orientation between the new cell and the previous cell in the plan.
   * @param lastI The index of the previous cell in the plan
   * @param i     The index of the cell to add to the plan
   * @param plan  The vector of poses that makes up the wavefront plan
   * @return Number of new planned cells
   */
  int addPose(int lastI, int i, std::vector<geometry_msgs::Pose>& plan);
  
    /**
   * @brief Sets the start index to the cell that startPose is contained in if it is a valid place to start.
   * @param startPose The pose that the robot at the start of the plan
   * @return True if the start index was set successfully
   */
  bool setStartIndex(geometry_msgs::Pose startPose);

  int findRobotCellStart(int startIndex);
  
  void printPlan(std::vector<int> poses);
public:
  /**
   * @brief Constructor.
   */
  Wavefront();

  /**
   * @brief Destructor.
   */
  ~Wavefront();

  /**
   * @brief Initializes the wavefront.
   * @param map Map data
   * @param ot  Occupancy threshold for the map grid
   * @param rw  Numer of wavefront cells that make up the width of the robot cell
   * @param rh  Numer of wavefront cells that make up the height of the robot cell
   * @return True if initialization was successful
   */
  bool initialize(CellGrid map, int ot, int cd);
  
  /**
   * @brief Propogates the wavefront and stores data in wavefront cells
   * @param startPose Pose of the robot at the start of the plan
   * @return True if the wavefront was propogated successfully
   */
  bool fill(geometry_msgs::Pose startPose);
  
  /**
   * @brief Creates a full coverage plan based on the propogated wavefront.
   * @param plan Stores the wavefront plan
   * @return True if a plan was possible
   */
  bool getPlan(std::vector<geometry_msgs::Pose> &plan);
  
  /**
   * @brief Resets data of cells that contains each pose in the given vector
   * @param cells Vector of poses that are within cells whose data should be reset
   */
  void resetCells(std::vector<geometry_msgs::Pose> cells);
  
  void findEmptyNeighbor(geometry_msgs::Pose &pose);
  
  /**
   * @brief Re-initializes the wavefront propogation
   */
  void reinit();
  
}; // Wavefront

} // namespace full_coverage

#endif // _WAVEFRONT_H_
