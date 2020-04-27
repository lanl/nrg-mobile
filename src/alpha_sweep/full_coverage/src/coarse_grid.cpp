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

#include "coarse_grid.h"

namespace full_coverage
{

CoarseGrid::CoarseGrid() :
  CoverageGrid(),
  coarseCells(),
  map(),
  occupancyThreshold(0),
  numBorderCells(0)
{
  // Do nothing
}

CoarseGrid::~CoarseGrid()
{
  // Do nothing
}

CoarseGrid::CoarseGrid(const CoarseGrid &cg)
  :CoverageGrid(cg)
{
  coarseCells = cg.coarseCells;
  map = cg.map;
  occupancyThreshold = cg.occupancyThreshold;
  numBorderCells = cg.numBorderCells;
  for(int i = 0; i < coarseCells.size(); i++)
  {
    cells.push_back(&coarseCells[i]);
  }
}

bool CoarseGrid::initialize(CellGrid& m, int ot, int nbc)
{
  if(ot <= 0)
  {
    ROS_ERROR("Coarse Grid: Occupancy threshold must be positive integer");
    return false;
  }
  if(nbc < 1)
  {
    ROS_ERROR("Coarse Grid: Num border cells must be 1 or more.");
  }
  numBorderCells = nbc;
  map = m;
  occupancyThreshold = ot;
  
  if(!CoverageGrid::initialize(m.grid_cells.size(), m.grid_cols))
    return false;
  
  return true;
}

bool CoarseGrid::initializeCells()
{
  coarseCells.resize(getSize(), CoarseCell());
  for(int i = 0; i<coarseCells.size(); i++)
  {
    coarseCells[i].initialize(i, map.grid_cells[i], occupancyThreshold);
    cells.push_back(&coarseCells[i]);
  }
  
  // Make sure things look right
  if(coarseCells.size() != getSize() || coarseCells.size() != cells.size())
  {
    ROS_ERROR("Coarse Grid: cannot initialize. check cell vectors");
    return false;
  }
  for(int i = 0; i<coarseCells.size(); i++)
  {
    if(coarseCells[i].getPosition() != i || cells[i]->getPosition() != i)
    {
      ROS_ERROR("Coarse Grid: wavecells were not properly initialized. Indexing is wrong");
      return false;
    }
  }
  
  return CoverageGrid::initializeCells();
}

int CoarseGrid::setNeighbors(int index, CellValue val)
{
  return coarseCells[index].setNeighbors(val);
}

void CoarseGrid::findNeighbors()
{
  setBorders();
  CoverageGrid::findNeighbors();
}

void CoarseGrid::setBorders()
{
  std::vector<int> borders;
  for(int i = 0; i<coarseCells.size(); i++)
  {
    if(OPEN_CELL == coarseCells[i].getValue())
    {
      bool newBorder = false;

      // get r,c of cell
      int r,c;
      r = 0;
      c = 0;
      getCoordinate(i, r, c);

      int row = r-numBorderCells;
      while(row <= r+numBorderCells && !newBorder)
      {
        int col = c-numBorderCells;
        while(col <= c+numBorderCells && !newBorder)
        {
          int ind;
          if(getIndex(ind,row,col))
          {
            if(OBSTRUCTED_CELL == coarseCells[ind].getValue() || UNKNOWN_CELL == coarseCells[ind].getValue())
            {
              borders.push_back(i);
              newBorder = true;
            }
          }
          else
          {
            borders.push_back(i);
            newBorder = true;
          }
          col++;
        }
        row++;
      }
    }
  }

  for(int i = 0; i<borders.size(); i++)
  {
    coarseCells[borders[i]].setValue(BORDER_CELL);
  }
}

} // namespace full_coverage

