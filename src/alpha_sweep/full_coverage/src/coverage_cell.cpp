#include "coverage_cell.h"

namespace full_coverage
{

CoverageCell::CoverageCell() :
  position(-1),
  value(UNKNOWN_CELL),
  cellPlanned(false),
  cellVisited(false),
  cellAttempted(false),
  cellReachable(false),
  bounds(),
  isInitialized(false),
  neighbors()
{
  // Do nothing
}

CoverageCell::~CoverageCell()
{
  // Do nothing
}

void CoverageCell::initialize(int pos)
{
  // Make sure variables are valid
  if(pos<0)
  {
    ROS_ERROR("Coverage Cell: Position must be positive integer");
    return;
  }
  
  // Initialize variables
  position = pos;
  
  // Find vertices
  setCorners();
}

void CoverageCell::addNeighbor(CoverageCell* nb)
{
  neighbors.push_back(nb);
}

void CoverageCell::clearNeighbors()
{
  neighbors.clear();
}

void CoverageCell::getNeighbors(std::vector<CoverageCell*>& nbs)
{
  nbs = neighbors; 
}

void CoverageCell::getUnplannedNeighbors(std::vector<CoverageCell*>& unbs)
{
  // Check neighbors vector and return any that are not planned
  unbs.clear();
  for(int i = 0; i<neighbors.size(); i++)
  {
    if(!neighbors[i]->isPlanned())
      unbs.push_back(neighbors[i]);
  }
}

CellValue CoverageCell::getValue()
{
  return value;
}

bool CoverageCell::setValue(CellValue val)
{
  value = val;
  return true;
}

int CoverageCell::getPosition()
{
  return position;
}

void CoverageCell::setPlanned()
{
  cellPlanned = true;
}

void CoverageCell::visit()
{
  cellAttempted = true;
  cellVisited = true;
}

void CoverageCell::setAttempted()
{
  cellAttempted = true;
}

bool CoverageCell::isPlanned()
{
  return cellPlanned;
}

bool CoverageCell::isPlanned(double percent)
{
  return cellPlanned;
}

bool CoverageCell::isVisited()
{
  return cellVisited;
}

bool CoverageCell::isVisited(double percent)
{
  return cellVisited;
}

bool CoverageCell::isAttempted()
{
  return cellAttempted;
}

bool CoverageCell::isAttempted(double percent)
{
  return cellAttempted;
}

void CoverageCell::setReachable(bool ir)
{
  cellReachable = ir;
}

bool CoverageCell::isReachable()
{
  return cellReachable;
}

bool CoverageCell::contains(geometry_msgs::Pose pose)
{
  // if the given pose is within the cell boundaries, return true
  return bounds.contains(pose);
}

geometry_msgs::Pose CoverageCell::getPose()
{
  // Get the center point of this cell in pose form.
  return bounds.getCenter();
}

std::vector<geometry_msgs::Pose> CoverageCell::getCorners()
{
  std::vector<geometry_msgs::Pose> corners;
  bounds.getCorners(corners);
  return corners;
}

void CoverageCell::setCorners(std::vector<geometry_msgs::Pose> allCorners)
{
  bounds.setBounds(allCorners);
}

double CoverageCell::getWidth()
{
  return bounds.getWidth();
}

} // namespace full_coverage

