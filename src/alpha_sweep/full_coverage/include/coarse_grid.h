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
* Authors: Alex von Sternberg and Meredith Symmank
*
* Description: Stores and calculates coarse_grid data for the full 
*  coverage planner.
*********************************************************************/

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
