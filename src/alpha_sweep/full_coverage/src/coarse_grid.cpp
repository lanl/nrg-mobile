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

