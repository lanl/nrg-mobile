#include "coarse_cell.h"

namespace full_coverage
{

CoarseCell::CoarseCell() :
  CoverageCell(),
  mapCell(),
  occupancyProb(0),
  occupancyThreshold(1)
{
  // Do nothing
}

CoarseCell::~CoarseCell()
{
  // Do nothing
}

void CoarseCell::initialize(int pos, SquareCell& mc, int ot)
{
  // Initialize variables
  occupancyThreshold = ot;
  mapCell = mc;
  
  // Call super initialize
  CoverageCell::initialize(pos);
 
  // Set occupancy values
  if(mapCell.occupancy_prob >= occupancyThreshold)
    setValue(OBSTRUCTED_CELL);
  else if(mapCell.occupancy_prob < 0)
    setValue(UNKNOWN_CELL);
  else
    setValue(OPEN_CELL);
  
  isInitialized = true;
}

void CoarseCell::setCorners()
{
  std::vector<geometry_msgs::Pose> verts;
  verts.push_back(mapCell.tl_vertex.pose);
  verts.push_back(mapCell.tr_vertex.pose);
  verts.push_back(mapCell.bl_vertex.pose);
  verts.push_back(mapCell.br_vertex.pose);
  CoverageCell::setCorners(verts);
}

int CoarseCell::setNeighbors(CellValue val)
{
  // Set the value of this cell's neighbors and return how many were set successfully
  int count = 0;
  for(int i = 0; i<neighbors.size(); i++)
  {
    if(neighbors[i]->setValue(val))
      count++;
  }
  return count;
}

} // namespace full_coverage

