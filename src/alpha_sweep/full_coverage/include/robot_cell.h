/*********************************************************************
*
*  © (or copyright) 2020. Triad National Security, LLC.
*  All rights reserved.
*  This program was produced under U.S. Government contract 
*  89233218CNA000001 for Los AlamosNational Laboratory (LANL), 
*  which is operated by Triad National Security, LLC for the U.S.
*  Department of Energy/National Nuclear Security Administration. 
*  All rights in the program are reserved by Triad National 
*  Security, LLC, and the U.S. Department of Energy/National Nuclear
*  Security Administration. The Government is granted for itself 
*  and others acting on its behalf a nonexclusive, paid-up, 
*  irrevocable worldwide license in this material to reproduce, 
*  prepare derivative works, distribute copies to the public, 
*  perform publicly and display publicly, and to permit others 
*  to do so.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alex von Sternberg
*
* Description: Holds and calculates wavefront data for one cell of 
*  the map.
*********************************************************************/

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
