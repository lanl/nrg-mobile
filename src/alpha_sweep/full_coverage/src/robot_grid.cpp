#include "robot_grid.h"

namespace full_coverage
{

RobotGrid::RobotGrid() :
  CoverageGrid(),
  robotCells(),
  coarseGrid(0),
  cellDim(0),
  secondAttempt(false)
{
  // do nothing
}

RobotGrid::~RobotGrid()
{
  // do nothing
}

RobotGrid::RobotGrid(const RobotGrid &rg, CoarseGrid *cg)
  : CoverageGrid(rg)
{
  if(cg)
  {
    robotCells = rg.robotCells;
    coarseGrid = cg;
    cellDim = rg.cellDim;
    for(int i = 0; i < robotCells.size(); i++)
    {
      cells.push_back(&robotCells[i]);
    }
  }
  else
  {
    ROS_ERROR("Cannot copy robot grid. Must be passed pointer to corresponding coarse grid.");
  }
}

bool RobotGrid::initialize(CoarseGrid* wg, int cd)
{
  if(cd <= 0)
  {
    ROS_ERROR("Robot Grid: Num wavefront cells per robot dim must be positive interger.");
    return false;
  }
  
  cellDim = cd;
  coarseGrid = wg;
  
  if(!CoverageGrid::initialize((wg->getNumCols()-cd+1)*(wg->getNumRows()-cd+1), wg->getNumCols()-cd+1))
    return false;

  return true;
}

bool RobotGrid::initializeCells()
{
  // Initialize robot cells
  robotCells.resize(getSize(),RobotCell());
  for(int i = 0; i<robotCells.size(); i++)
  {
    int r = 0;
    int c =0;
    getCoordinate(i,r,c);
    std::vector<CoverageCell*> inh;
    for(int row = r; row<r+cellDim; row++)
    {
      for(int col = c; col<c+cellDim; col++)
      {
        int index = 0;
        if(coarseGrid->getIndex(index,row,col))
        {
          CoverageCell* temp = nullptr;
          if(coarseGrid->getCell(index,temp))
          {
            inh.push_back(temp);
          }
          else
          {
            ROS_ERROR("Robot Grid::initializeCells tried to retrieve invalid cell"); 
            return false;
          }
        }
        else
        {
          ROS_ERROR("Robot Grid::initalizeCells tried to get invalid index");
          return false;
        }
      }
    }
    robotCells[i].initialize(i, inh);
    cells.push_back(&robotCells[i]);
    
    // update cell value it knows if its occupied
    robotCells[i].updateValue();
  }
  
  // Make sure everything looks good
  if(robotCells.size() != getSize() || robotCells.size() != cells.size())
  {
    ROS_ERROR("Robot Grid: cannot initialize. check cell vectors");
    return false; 
  }
  for(int i = 0; i<robotCells.size(); i++)
  {
    if(robotCells[i].getPosition() != i || cells[i]->getPosition() != i)
    {
      ROS_ERROR("Robot Grid: robotcells were not properly initialized. Indexing is wrong");
      return false;
    }
  }
  
  return CoverageGrid::initializeCells();
}

void RobotGrid::findNeighbors()
{
  // Set up neighbors vectors for coverage_grid cells
  std::vector<CoverageCell*> neighbors; // vector of valid neighbors to be passed to cell
  for(int i = 0; i<robotCells.size(); i++)
  {
    robotCells[i].clearNeighbors();
    if(OPEN_CELL == robotCells[i].getValue())
    {
      neighbors.clear();
      int r,c;
      r = 0;
      c = 0;
      getCoordinate(i, r, c);
      for(int row = r-cellDim; row<=r+cellDim; row++)
      {
        for(int col = c-cellDim; col<=c+cellDim; col++)
        {
          if((row!=r && col == c) || (row == r && col != c)) // restrict to straight line motion for cleaner path
            addNeighbor(neighbors,row,col); // try adding possible neighbor
        }
      }
      for(int j =0; j<neighbors.size(); j++)
      {
        robotCells[i].addNeighbor(neighbors[j]); // add the valid neighbors that we found
      }
    }
  }
}

bool RobotGrid::findOpenNeighbor(int &index, double openFraction, bool allowUnknown)
{
  // Check if cell has any unobstructed neighbors
  if(index < 0 || index >= getSize())
  {
    ROS_ERROR("Robot Grid: Index passed to findOpenNeighbor is not valid");
    return false;
  }

  // Find the closest open cell
  bool found = false;        // true if we have found the closest open cell
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be open
  checked.push_back(index);  // put the current cell on the checked list
  std::vector<int> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.push_back(index); // we start propogation from the cell we got stuck at
  bool foundNonObs = false;
  while(!found && neighbors.size()!=0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i<numOld && !found) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<int> newNeighbors;
      newNeighbors.push_back(neighbors[i]);
      getSurroundingCells(newNeighbors, foundNonObs, allowUnknown); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j<newNeighbors.size() && !found) // check all new neighbors unless one of them is found to be a valid hop
      {
        if(std::find(checked.begin(), checked.end(), newNeighbors[j]) == checked.end()) // if the new neighbor has not already been checked
        {
          neighbors.push_back(newNeighbors[j]); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
          checked.push_back(newNeighbors[j]); // add this neighbor to the already checked list
          bool discriminator = OPEN_CELL == robotCells[newNeighbors[j]].getValue() && 
                               !robotCells[newNeighbors[j]].isVisited(openFraction) && 
                               (!robotCells[newNeighbors[j]].isAttempted(openFraction) || secondAttempt); //if it is open and not visited, it is an open neighbor
          if(allowUnknown)
            discriminator = discriminator || UNKNOWN_CELL == robotCells[newNeighbors[j]].getValue(); // also true if cell is completely unknown 
          if(discriminator)
          {
            index = newNeighbors[j];
            found = true;
          }
        }
        j++;
      }
      i++;
    }
    if(numOld == 1) // delete the neighbors who ran through the algorithm in the last iteration so we don't keep rechecking neighbors
      neighbors.erase(neighbors.begin());
    else
      neighbors.erase(neighbors.begin(), neighbors.begin()+numOld);
  }

  if(!found)
  {
    ROS_ERROR("Robot Grid: CoverageGrid::findOpenNeighbor could not find an empty neighbor");
    return false;
  }
  else
    return true;
}

void RobotGrid::getSurroundingCells(std::vector<int> &ind, bool &foundNonObs, bool allowUnknown)
{
  std::vector<int> startVals = ind;
  ind.clear();
  for(int i = 0; i<startVals.size(); i++)
  {
    int r,c;
    if(getCoordinate(startVals[i],r,c))
    {
      int tempIndex;
      if(getIndex(tempIndex, r-1, c))
      {
        bool nonObs = robotCells[tempIndex].getValue() != OBSTRUCTED_CELL;
        if(!allowUnknown)
          nonObs = nonObs && robotCells[tempIndex].getValue() != UNKNOWN_CELL;
        if(!foundNonObs || nonObs)
          ind.push_back(tempIndex);
        if(nonObs)
          foundNonObs = true;
      }
      if(getIndex(tempIndex, r+1, c))
      {
        bool nonObs = robotCells[tempIndex].getValue() != OBSTRUCTED_CELL;
        if(!allowUnknown)
          nonObs = nonObs && robotCells[tempIndex].getValue() != UNKNOWN_CELL;
        if(!foundNonObs || nonObs)
          ind.push_back(tempIndex);
        if(nonObs)
          foundNonObs = true;
      }
      if(getIndex(tempIndex, r, c-1))
      {
        bool nonObs = robotCells[tempIndex].getValue() != OBSTRUCTED_CELL;
        if(!allowUnknown)
          nonObs = nonObs && robotCells[tempIndex].getValue() != UNKNOWN_CELL;
        if(!foundNonObs || nonObs)
          ind.push_back(tempIndex);
        if(nonObs)
          foundNonObs = true;
      }
      if(getIndex(tempIndex, r, c+1))
      {
        bool nonObs = robotCells[tempIndex].getValue() != OBSTRUCTED_CELL;
        if(!allowUnknown)
          nonObs = nonObs && robotCells[tempIndex].getValue() != UNKNOWN_CELL;
        if(!foundNonObs || nonObs)
          ind.push_back(tempIndex);
        if(nonObs)
          foundNonObs = true;
      }
    }
  }
}

bool RobotGrid::updateValue(int index)
{
  if(index>=0 && index<robotCells.size())
  {
    robotCells[index].updateValue();
    return true;
  }
  else
  {
    ROS_ERROR("RobotGrid::updateValue passed invalid index. Cannot update value.");
    return false;
  }
}

MapDirection RobotGrid::getWallDirection(int index)
{
  MapDirection dir = MAP_NORTH;
  int r,c;
  if(!getCoordinate(index,r,c))
    return dir;

  // If we are already next to a wall, pick the direction that puts the wall on our left.
  if(isWallAhead(r,c,MAP_NORTH))
    return MAP_EAST;
  if(isWallAhead(r,c,MAP_EAST))
    return MAP_SOUTH;
  if(isWallAhead(r,c,MAP_SOUTH))
    return MAP_WEST;
  if(isWallAhead(r,c,MAP_WEST))
    return MAP_NORTH;

  // If we are not next to a wall, find the direction that points to the closest wall.
  bool found = false;
  int dist = 1;
  while(!found && dist < 1000000)
  {
    dir = MAP_NORTH;
    if(isWall(r,c,dist,dir))
      return dir;
    dir = MAP_EAST;
    if(isWall(r,c,dist,dir))
      return dir;
    dir = MAP_SOUTH;
    if(isWall(r,c,dist,dir))
      return dir;
    dir = MAP_WEST;
    if(isWall(r,c,dist,dir))
      return dir;
    dist++;
  }

  return dir;
}

bool RobotGrid::isWall(int r, int c, int dist, MapDirection dir)
{
  int i,ri,ci;
  getAheadInc(ri,ci,dir);
  if(!getIndex(i,r+ri*dist,c+ci*dist) || OPEN_CELL != robotCells[i].getValue())
    return true;
  return false;
}

bool RobotGrid::findWall(int index, MapDirection dir, int& val, MapDirection &newDir, bool &found)
{
  // Set new values to current values
  newDir = dir;
  val = index;
  found = false;
  int r,c,i;
  if(getCoordinate(index, r, c))
  {
    int rInc, cInc;
    int row = r;
    int col = c;
    getAheadInc(rInc,cInc,dir);
    int count = 0;

    // check ahead of us until we have checked a full robot length or found a wall
    while(count<cellDim && !found)
    {
      if(getIndex(i,row,col))
      {
        if(isWallAhead(row,col,dir)) // Next cell ahead is a wall. Turn right and return current location.
        {
          found = true;
          newDir = getRight(dir);
        }
        else if(isWallLeft(row,col,dir)) // A wall is to our left. Return current direction and location.
        {
          found = true;
        }
      }
      else // Current index is invalid (map edges are considered a wall). Turn right and return previous (valid) location
      {
        row-=rInc;
        col-=cInc;
        found = true;
        newDir = getRight(dir);
      }
      if(!found) // If we haven't found a wall, move forward one cell
      {
        row+=rInc;
        col+=cInc;
        count++;
      }
    }
    // Get index of current row/col to return
    getIndex(val,row,col);
    return true;
  }
  else // We were passed an invalid index
  {
    return false;
  }
}

bool RobotGrid::leaveHall(int index, MapDirection dir, int &val, MapDirection &newDir)
{
  // Special case where robot is at dead end with width of robot
  // Should be called after robot has turned around to move to the next position without a wall on left so we can exit leaveHall
  bool done = false;
  int r,c,i;
  if(getCoordinate(index, r, c))
  {
    int rIncA, cIncA;
    getAheadInc(rIncA,cIncA,dir);
    while(!done)
    {
      if(getIndex(i,r,c))
      {
        // Decide if there is a wall to the left or ahead
        bool wallL, wallA;
        wallL = isWallLeft(r,c,dir);
        wallA = !isOpenAhead(r,c,dir);
     
        // if there is no wall to the left stop and return to main algorithm
        if(!wallL)
        {
          val = i;
          newDir = dir;
          done = true;
        }
        // if there is a wall to left and we the cell we are on is not planned or visited, return to original algorithm
        else if(!robotCells[i].isPlanned() && !robotCells[i].isVisited(0.3))
        {
          val = i;
          newDir = dir;
          done = true;
        }
        // if there is a wall to the left and in front, return false. algorithm doesn't work in this situation
        else if(wallL && wallA)
          return false; 
      }
      else
        return false;
      if(!done)
      {
        r+=rIncA;
        c+=cIncA;
      }
    }
    return true;
  }
  else
    return false;
}

bool RobotGrid::getBorderNeighbor(int index, MapDirection dir, int& val, MapDirection &newDir)
{
  int r,c,i;
  if(getCoordinate(index, r, c))
  {
    int rInc, cInc;
    getAheadInc(rInc,cInc,dir);
    int count = 0;
    bool done = false;
    int row = r;
    int col = c;
    bool turnLeft = false;
    bool turnRight = false;
    while(count<=cellDim && !done)
    {
      if(getIndex(i,row,col))
      {
        if(!isWallLeft(row,col,dir))
        {
          turnLeft = true;
          done = true;
        }
        else if(isWallAhead(row,col,dir))
        {
          turnRight = true;
          done = true;
        }
      }
      else
      {
        done = true;
      }
      if(!done)
      {
        count++;
        if(count<=cellDim)
        {
          row+=rInc;
          col+=cInc;
        }
      }
    }
    if(count<1)
    {
      // Loop exited on first pass so we are not currently along a wall
      if(turnLeft)
      {
        // return index of cell to the left and change direction to the left
        int rInc,cInc;
        getLeftInc(rInc,cInc,dir);
        if(!getIndex(val,row+rInc,col+cInc))
        {
          // we somehow ended up at an invalid index. this should not happen.
          ROS_ERROR("Robot Grid::getBorderNeighbor ended up at invalid index. Returning current index.");
          val = index;
        }
        else if(robotCells[val].isPlanned() || robotCells[val].isVisited() || robotCells[val].getValue() != OPEN_CELL)
        {
          // can't move left because its planned or obstructed
          val = index;
        }
        newDir = getLeft(dir);
      }
      else if(turnRight)
      {
        // return index of cell to the right and change direction to the right 
        int rInc,cInc;
        getLeftInc(rInc,cInc,dir);
        if(!getIndex(val,row-rInc,col-cInc))
        {
          // there is no cell to the right. just turn right.
          if(!getIndex(val,row,col))
          {
            val = index;
          }
        }
        else if(robotCells[val].isPlanned() || robotCells[val].isVisited() || robotCells[val].getValue() != OPEN_CELL)
        {
          // can't move right because its planned or obstructed
          if(!getIndex(val,row,col))
          {
            val = index;
          }
        }
        else
        newDir = getRight(dir);
      }
      else
      {
        // cell is invalid. return current index and dir.
        val = index;
        newDir = dir;
      }
    }
    else
    {
      // we moved forward some. keep same direction but return index of cell we moved forward to.
      newDir = dir;
      if(!getIndex(val,row,col))
      {
        // we somehow ended up at an invalid index. this should not happen.
        ROS_ERROR("Robot Grid::getBorderNeighbor ended up at invalid index. Returning current index.");
        val = index;
      }
    }
    return true;
  }
  else
    return false;
}

MapDirection RobotGrid::getLeft(MapDirection dir)
{
  switch(dir)
  {
    case MAP_NORTH:
      return MAP_WEST;
    case MAP_EAST:
      return MAP_NORTH;
    case MAP_SOUTH:
      return MAP_EAST;
    case MAP_WEST:
      return MAP_SOUTH;
    default:
      ROS_ERROR("Robot Grid::getLeft received invalid direction.");
      return MAP_NORTH;      
  }
}

MapDirection RobotGrid::getRight(MapDirection dir)
{
  switch(dir)
  {
    case MAP_NORTH:
      return MAP_EAST;
    case MAP_EAST:
      return MAP_SOUTH;
    case MAP_SOUTH:
      return MAP_WEST;
    case MAP_WEST:
      return MAP_NORTH;
    default:
      ROS_ERROR("Robot Grid::getRight received invalid direction.");
      return MAP_NORTH;     
  }
}

bool RobotGrid::isWallLeft(int r, int c, MapDirection currentDir)
{
  int rInc,cInc;
  getLeftInc(rInc,cInc,currentDir);
  
  return isPlannedLeft(r+rInc,c+cInc,currentDir);
}

void RobotGrid::getLeftInc(int &ri, int &ci, MapDirection dir)
{
  ri = 0;
  ci = 0;
  switch(dir)
  {
    case MAP_NORTH:
      getAheadInc(ri,ci,MAP_WEST);
      break;
    case MAP_EAST:
      getAheadInc(ri,ci,MAP_NORTH);
      break;
    case MAP_SOUTH:
      getAheadInc(ri,ci,MAP_EAST);
      break;
    case MAP_WEST:
      getAheadInc(ri,ci,MAP_SOUTH);
      break;
    default:
      ROS_ERROR("Robot Grid::getLeftInc received invalid direction. Cannot get neighbor.");
  }
}

void RobotGrid::getAheadInc(int &ri, int &ci, MapDirection dir)
{
  ri = 0;
  ci = 0;
  switch(dir)
  {
    case MAP_NORTH:
      ri = -1;
      break;
    case MAP_EAST:
      ci = 1;
      break;
    case MAP_SOUTH:
      ri = 1;
      break;
    case MAP_WEST:
      ci = -1;
      break;
    default:
      ROS_ERROR("Robot Grid::getAheadInc received invalid direction. Cannot get neighbor.");
  }
}

bool RobotGrid::isWallAhead(int r, int c, MapDirection currentDir)
{
  int rInc,cInc;
  getAheadInc(rInc,cInc,currentDir);
  
  return isPlannedAhead(r+rInc,c+cInc,currentDir);
}

bool RobotGrid::isOpenAhead(int r, int c, MapDirection currentDir)
{
  int rInc,cInc,index;
  getAheadInc(rInc,cInc,currentDir);

  if(getIndex(index,r+rInc,c+cInc))
  {
    if(robotCells[index].getValue() == OPEN_CELL)
      return true;
    else
      return false;
  }
  else
    return false;

  return isPlannedAhead(r+rInc,c+cInc,currentDir);
}

bool RobotGrid::isPlannedLeft(int r, int c, MapDirection currentDir)
{
  int index = 0;
  bool planned, attempted, visited;
  if(getIndex(index,r,c))
  {
    switch(currentDir)
    {
      case MAP_NORTH:
        planned = robotCells[index].isPlanned(cellDim*(cellDim-1));
        attempted = robotCells[index].isAttempted(cellDim*(cellDim-1));
        visited = robotCells[index].isVisited(cellDim*(cellDim-1));
        break;
      case MAP_EAST:
        planned = robotCells[index].isPlanned(0);
        attempted = robotCells[index].isAttempted(0);
        visited = robotCells[index].isVisited(0);
        break;
      case MAP_SOUTH:
        planned = robotCells[index].isPlanned(cellDim-1);
        attempted = robotCells[index].isAttempted(cellDim-1);
        visited = robotCells[index].isVisited(cellDim-1);
        break;
      default:
        planned = robotCells[index].isPlanned(cellDim*cellDim-1);
        attempted = robotCells[index].isAttempted(cellDim*cellDim-1);
        visited = robotCells[index].isVisited(cellDim*cellDim-1);
        break;
    }
    if(visited || (!secondAttempt && attempted) || planned || robotCells[index].getValue() != OPEN_CELL)
    {
      return true;
    }
    else
      return false;
  }
  else
    return true;
}

bool RobotGrid::isPlannedAhead(int r, int c, MapDirection currentDir)
{
  int index = 0;
  bool planned, attempted, visited;
  if(getIndex(index,r,c))
  {
    switch(currentDir)
    {
      case MAP_NORTH:
        planned = robotCells[index].isPlanned(0);
        attempted = robotCells[index].isAttempted(0);
        visited = robotCells[index].isVisited(0);
        break;
      case MAP_EAST:
        planned = robotCells[index].isPlanned(cellDim-1);
        attempted = robotCells[index].isAttempted(cellDim-1);
        visited = robotCells[index].isVisited(cellDim-1);
        break;
      case MAP_SOUTH:
        planned = robotCells[index].isPlanned(cellDim*cellDim-1);
        attempted = robotCells[index].isAttempted(cellDim*cellDim-1);
        visited = robotCells[index].isVisited(cellDim*cellDim-1);
        break;
      default:
        planned = robotCells[index].isPlanned(cellDim*(cellDim-1));
        attempted = robotCells[index].isAttempted(cellDim*(cellDim-1));
        visited = robotCells[index].isVisited(cellDim*(cellDim-1));
        break;
    }
    if(visited || (!secondAttempt && attempted) || planned || robotCells[index].getValue() != OPEN_CELL)
    {
      return true;
    }
    else
      return false;
  }
  else
    return true;
}

bool RobotGrid::setPlanned(int index, int &numChanged)
{
  numChanged = 0;
  if(index>=0 && index<robotCells.size())
  {
    robotCells[index].setPlanned(numChanged);
    return true;
  }
  else
  {
    ROS_ERROR("RobotGrid::setPlanned passed invalid index. Cannot set cell as planned.");
    return false;
  }
}

//bool RobotGrid::visit(int index)
//{
//  if(index>=0 && index<robotCells.size())
//  {
//    robotCells[index].visit();
//    return true;
//  }
//  else
//  {
//    ROS_ERROR("RobotGrid::visit passed invalid index. Cannot set cell as visited.");
//    return false;
//  }
//}

//int RobotGrid::getIndex(geometry_msgs::Pose pose)
//{
//  if(!isInitialized)
//  {
//    ROS_ERROR("Robot Grid: Cannot get index of pose before cells are initialized.");
//    return -1;
//  }
//
//  // Return the index of the cell that contains pose
//  std::vector<int> options;
//  for(int i = 0; i<robotCells.size(); i++)
//  {
//    if(robotCells[i].contains(pose))
//      options.push_back(i);
//  }
//
//  // If the pose was not found within a cell, return -1
//  if(options.size()<=0)
//  {
//    ROS_ERROR("Robot Grid: Pose passed in getIndex is not contained within any robot cells.");
//    return -1;
//  }
//
//  int bestI = options[0];
//  geometry_msgs::Pose temp = robotCells[bestI].getPose();
//  double best = getDistance(temp,pose);
//  for(int i = 1; i<options.size(); i++)
//  {
//    temp = robotCells[options[i]].getPose();
//    double dist = getDistance(temp,pose);
//    if(dist<best)
//    {
//      best = dist;
//      bestI = options[i];
//    }
//  }
//  return bestI;
//}

void RobotGrid::getPositions(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds)
{
  inds.clear();
  std::vector<std::vector<int>> options;
  options.resize(poses.size());
  for(int i = 0; i < cells.size(); i++)
  {
    for(int j = 0; j < poses.size(); j++)
    {
      if(cells[i]->contains(poses[j]))
      {
        options[j].push_back(i);
      }
    }
  }

  for(int i = 0; i < options.size(); i++)
  {
    if(options.at(i).size() > 0)
    {
      int bestI = options.at(i).at(0);
      geometry_msgs::Pose temp = robotCells[bestI].getPose();
      double best = getDistance(temp,poses[i]);
      for(int j = 1; j<options.at(i).size(); j++)
      {
        temp = robotCells[options.at(i).at(j)].getPose();
        double dist = getDistance(temp,poses[i]);
        if(dist<best)
        {
          best = dist;
          bestI = options.at(i).at(j);
        }
      }
      inds.push_back(bestI);
    }
  }
}

//bool RobotGrid::visit(geometry_msgs::Pose pose)
//{
//  return visit(getIndex(pose));
//}

int RobotGrid::getHeight()
{
  return cellDim;
}

int RobotGrid::getWidth()
{
  return cellDim;
}

bool RobotGrid::getCoarseIndex(int &ind)
{
  if(ind>=0 && ind<robotCells.size())
  {
    return robotCells[ind].getCoarseIndex(ind);
  }
  else
  {
    ROS_ERROR("RobotGrid: getCoarseIndex passed invalid index.");
    return false;
  }
}

void RobotGrid::setSecondAttempt(bool sec)
{
  secondAttempt = sec;
}

} // namespace full_coverage

