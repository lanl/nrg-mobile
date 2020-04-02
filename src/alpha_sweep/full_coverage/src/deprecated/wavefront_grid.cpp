#include "wavefront_grid.h"

namespace full_coverage
{

WavefrontGrid::WavefrontGrid() :
  CoverageGrid(),
  waveCells(),
  map(),
  occupancyThreshold(0),
  numBorderCells(0)
{
  // Do nothing
}

WavefrontGrid::~WavefrontGrid()
{
  // Do nothing
}

bool WavefrontGrid::initialize(CellGrid& m, int ot, int nbc)
{
  if(ot <= 0)
  {
    ROS_ERROR("Wavefront Grid: Occupancy threshold must be positive integer");
    return false;
  }
  if(nbc < 1)
  {
    ROS_ERROR("Wavefront Grid: Num border cells must be 1 or more.");
  }
  numBorderCells = nbc;
  map = m;
  occupancyThreshold = ot;
  
  if(!CoverageGrid::initialize(m.grid_cells.size(), m.grid_cols))
    return false;
  
  return true;
}

bool WavefrontGrid::initializeCells()
{
  waveCells.resize(getSize(), WavefrontCell());
  for(int i = 0; i<waveCells.size(); i++)
  {
    waveCells[i].initialize(i, map.grid_cells[i], occupancyThreshold);
    cells.push_back(&waveCells[i]);
  }
  
  // Make sure things look right
  if(waveCells.size() != getSize() || waveCells.size() != cells.size())
  {
    ROS_ERROR("Wavefront Grid: cannot initialize. check cell vectors");
    return false;
  }
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(waveCells[i].getPosition() != i || cells[i]->getPosition() != i)
    {
      ROS_ERROR("Wavefront Grid: wavecells were not properly initialized. Indexing is wrong");
      return false;
    }
  }
  
  return CoverageGrid::initializeCells();
}

void WavefrontGrid::print()
{
  /********************************************************************************/
  /** This code block will print the wavefront. Use only for debugging purposes. **/
  /********************************************************************************/
  int grid_cols = getNumCols();
  int grid_rows = getNumRows();
  int index;

  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  full_coverage::SquareCell* cell;
  
  std::cout << "-----------------------------------------------------------------" << std::endl;
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (waveCells[index].getValue() == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "| |" <<default_col<<"|";
      else if(waveCells[index].isPlanned())
      {
        if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
          std::cout << "|  "<< color_red << waveCells[index].getValue() << default_col << "|"; 
        else if(waveCells[index].getValue()<100 && waveCells[index].getValue()>=0)
          std::cout << "| "<< color_red << waveCells[index].getValue() << default_col << "|"; 
        else
          std::cout << "|"<< color_red << waveCells[index].getValue() << default_col << "|"; 
      }
      else
      {
        if(waveCells[index].getValue()<10 && waveCells[index].getValue()>=0)
          std::cout << "|  "<< waveCells[index].getValue() <<"|"; 
        else if(waveCells[index].getValue()<100 && waveCells[index].getValue()>=0)
          std::cout << "| "<< waveCells[index].getValue() <<"|"; 
        else
          std::cout << "|"<< waveCells[index].getValue() <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
}

int WavefrontGrid::setNeighbors(int index, int wave)
{
  return waveCells[index].setNeighbors(wave);
}

void WavefrontGrid::findNeighbors()
{
  setBorders();
  CoverageGrid::findNeighbors();
}

void WavefrontGrid::setBorders()
{
  std::vector<int> borders;
  for(int i = 0; i<waveCells.size(); i++)
  {
    if(WAVE_OBSTRUCTION != waveCells[i].getValue())
    {
      bool newBorder = false;

      // get r,c of cell
      int r,c = 0;
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
            if(WAVE_OBSTRUCTION == waveCells[ind].getValue())
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
    waveCells[borders[i]].setValue(WAVE_BORDER);
    waveCells[borders[i]].setInitValue(WAVE_BORDER);
  }
}

} // namespace full_coverage

