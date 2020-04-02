#include "wavefront_cell.h"

namespace full_coverage
{

WavefrontCell::WavefrontCell() :
  CoverageCell(),
  mapCell(),
  occupancyProb(0),
  occupancyThreshold(1)
{
  // Do nothing
}

WavefrontCell::~WavefrontCell()
{
  // Do nothing
}

void WavefrontCell::initialize(int pos, SquareCell& mc, int ot)
{
  // Initialize variables
  occupancyThreshold = ot;
  mapCell = mc;
  
  // Call super initialize
  CoverageCell::initialize(pos);
 
  // Set occupancy values
  if(mapCell.occupancy_prob > occupancyThreshold)
    setValue(WAVE_OBSTRUCTION);
  else
    setValue(WAVE_INIT);
  setInitValue(getValue());
  
  isInitialized = true;
}

void WavefrontCell::setCorners()
{
  std::vector<geometry_msgs::Pose> verts;
  verts.push_back(mapCell.tl_vertex.pose);
  verts.push_back(mapCell.tr_vertex.pose);
  verts.push_back(mapCell.bl_vertex.pose);
  verts.push_back(mapCell.br_vertex.pose);
  CoverageCell::setCorners(verts);
}

bool WavefrontCell::setValue(int val)
{
  // Set the cell to the wave value
  // if the cell has not been changed since initialization or if the value is reinitializing the cell (don't set it if it has already been assigned a wave value)
  // in any case, do not set the value if this cell is an obstruction (this data comes directly from the map and should not be changed)
  if((WAVE_INIT == getValue() || WAVE_INIT == val) && WAVE_OBSTRUCTION != getValue() && WAVE_BORDER !=getValue())
  {
    CoverageCell::setValue(val);
    return true;
  }
  else
  {
    return false;
  }
}

int WavefrontCell::setNeighbors(int wave)
{
  // Set the wave value of this cell's neighbors and return how many were set successfully
  int count = 0;
  for(int i = 0; i<neighbors.size(); i++)
  {
    if(neighbors[i]->setValue(wave))
      count++;
  }
  return count;
}

void WavefrontCell::reset()
{
  CoverageCell::reset();
  
  if(initValue != WAVE_INIT)
    CoverageCell::setValue(initValue);
  else if(occupancyProb > occupancyThreshold)
    CoverageCell::setValue(WAVE_OBSTRUCTION);
  else
    CoverageCell::setValue(WAVE_INIT);
}

} // namespace full_coverage

