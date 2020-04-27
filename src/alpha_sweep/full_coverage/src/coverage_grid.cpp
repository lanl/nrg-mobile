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
* Description: Stores and calculates coverage_grid data for the full
*  coverage planner.
*********************************************************************/

#include "coverage_grid.h"

namespace full_coverage
{

CoverageGrid::CoverageGrid() :
  mapSize(0),
  numRows(0),
  numCols(0),
  gridInit(false),
  isInitialized(false),
  cells()
{
  // Do nothing
}

CoverageGrid::~CoverageGrid()
{
  // Do nothing
}

CoverageGrid::CoverageGrid(const CoverageGrid &cg)
{
  mapSize = cg.mapSize;
  numRows = cg.numRows;
  numCols = cg.numCols;
  gridInit = cg.gridInit;
  isInitialized = cg.isInitialized;
  cells.clear();
}

bool CoverageGrid::initialize(int mSize, int nCols)
{
  // Set map variables
  mapSize = mSize;
  numCols = nCols;
  
  // Check for invalid parameters
  if(numCols <= 0)
  {
    ROS_ERROR("Coverage Grid: Grid must have at least one column.");
    return false;
  }
  else if(mapSize % numCols != 0)
  {
    ROS_ERROR("Coverage Grid: Grid vector size must be divisible by the number of columns.");
    return false;
  }

  // Calculate number of rows
  numRows = mapSize/numCols;
  
  gridInit = true;
 
  // Initialize cells
  if(!initializeCells())
    return false;
    
  // Figure out which cells are neighbors
  findNeighbors();
  
  // Make sure variables were initialized correctly
  if(cells.size() != mapSize)
  {
    ROS_ERROR("Coverage Grid: cells vector should contain a cell for every spot in the map. Cannot initailize Coverage grid.");
    return false;
  }
  
  // Set init flag
  isInitialized = true;
  
  return true;
}

bool CoverageGrid::initializeCells()
{
  if(cells.size() <=0)
  {
    ROS_ERROR("Coverage Grid: Must initialize cells vector to use coverage_grid");
    return false;
  }
  else
  {
    return true;
  }
}

bool CoverageGrid::hop(int &index)
{
  std::cout << "hopping" << std::endl;
  if(index < 0 && index >= cells.size())
  {
    ROS_ERROR("Coverage Grid: Cannot hop in CoverageGrid::hop. Cell index invalid.");
    return false;
  }
  
  // Find the closest unplanned cell
  bool found = false;        // true if we have found the closest unplanned cell
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be unplanned
  checked.push_back(index);  // put the current cell on the list since it should hop back to itself
  std::vector<CoverageCell*> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.push_back(cells[index]); // we start propogation from the cell we got stuck at
  while(!found && neighbors.size()!=0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i<numOld && !found) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<CoverageCell*> newNeighbors;
      neighbors[i]->getNeighbors(newNeighbors); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j<newNeighbors.size() && !found) // check all new neighbors unless one of them is found to be a valid hop
      {
        if(std::find(checked.begin(), checked.end(), newNeighbors[j]->getPosition()) == checked.end()) // if the new neighbor has not already been checked
        {
          int r, c;
          getCoordinate(newNeighbors[j]->getPosition(), r, c);
          neighbors.push_back(newNeighbors[j]); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
          checked.push_back(newNeighbors[j]->getPosition()); // add this neighbor to the already checked list
          if(!newNeighbors[j]->isPlanned() && !newNeighbors[j]->isVisited(0.0) && newNeighbors[j]->isReachable())  // if it has not been planned or visited, we found a valid hop
          {
            index = newNeighbors[j]->getPosition();
            found = true;
            std::cout << "found valid hop" << std::endl;
            newNeighbors[j]->isVisited(0.0);
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
  if(!found && neighbors.size()==0) // the algorithm failed (this means we have already planned all poses and didn't need to hop in the first place)
  {
    ROS_INFO("Coverage Grid: We ran out of neighbors. Hop was called when all poses had already been planned");
  }
  return found;
}

bool CoverageGrid::findReachable(int index)
{
  if(index < 0 && index >= cells.size())
  {
    ROS_ERROR("Coverage Grid: Cannot findReachable. Cell index invalid.");
    return false;
  }
  cells[index]->setReachable(true);

  // Find the set all neighbors and neighbors neigbors, etc to reachable
  std::vector<int> checked;  // vector of indices of cells that we have already checked to be unplanned
  checked.push_back(index);  // put the current cell on the list since it should hop back to itself
  std::vector<CoverageCell*> neighbors; // vector of neighbors propogating outward from original cell
  neighbors.push_back(cells[index]); // we start propogation from the cell we got stuck at
  while(neighbors.size()!=0) // keep going until we find a valid hop, or we have checked all valid cells and therefore neighbors is empty
  {
    int numOld = neighbors.size(); // number of neighbors in the previous iteration
    int i = 0; // iteration of old neighbors
    while(i<numOld) // iterate all old neighbors unless we find a valid cell to hop to
    {
      std::vector<CoverageCell*> newNeighbors;
      neighbors[i]->getNeighbors(newNeighbors); // neighbors of our old neighbors (this is how we propogate outward)
      int j = 0; // iteration of new neighbors
      while(j<newNeighbors.size()) // check all new neighbors unless one of them is found to be a valid hop
      {
        if(std::find(checked.begin(), checked.end(), newNeighbors[j]->getPosition()) == checked.end()) // if the new neighbor has not already been checked
        {
          int r, c;
          getCoordinate(newNeighbors[j]->getPosition(), r, c);
          neighbors.push_back(newNeighbors[j]); // move the new neighbor to the old neighbors list so that in the next iteration we check its neighbors
          checked.push_back(newNeighbors[j]->getPosition()); // add this neighbor to the already checked list
          newNeighbors[j]->setReachable(true);
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
  return true;
}

void CoverageGrid::findNeighbors()
{
  // Set up neighbors vectors for coverage_grid cells
  std::vector<CoverageCell*> neighbors; // vector of valid neighbors to be passed to cell
  for(int i = 0; i<cells.size(); i++)
  {
    cells[i]->clearNeighbors();
    if(OPEN_CELL == cells[i]->getValue()) // cell does not need neighbors if it is not available for planning
    {
      neighbors.clear();
      int r,c;
      r = 0;
      c= 0;
      getCoordinate(i, r, c);
      addNeighbor(neighbors,r+1,c); // try adding possible neighbor that sits below this cell
      addNeighbor(neighbors,r-1,c); // try adding possible neighbor that sits above this cell
      addNeighbor(neighbors,r,c+1); // try adding possible neighbor that sits to the right
      addNeighbor(neighbors,r,c-1); // try adding possible neighbor that sits to the left
      for(int j =0; j<neighbors.size(); j++)
      {
        cells[i]->addNeighbor(neighbors[j]); // add the valid neighbors that we found
      }
    }
  }
}

void CoverageGrid::addNeighbor(std::vector<CoverageCell*> &neighbors, int r, int c)
{
  // add a neighbor to the neighbors array if (r,c) is valid neighbor
  int index = 0;
  if(getIndex(index, r, c)) // false if (r,c) is not within map
  {
    if(OPEN_CELL == cells[index]->getValue()) // only add neighbor if it is available for planning
    {
      neighbors.push_back(cells[index]);
    }
  }
}

bool CoverageGrid::getCoordinate(int index, int &r, int &c)
{
  //return the coordinate corresponding to index if it is withing the map
  if(!gridInit)
  {
    ROS_ERROR("Coverage Grid: Cannot get coordinate before grid dimensions are set.");
    return false;
  }

  if(index < 0 || index >= mapSize)
  {
    ROS_ERROR("Coverage Grid: Index out of range. Cannot get coordinate");
    return false;
  }

  c = index % numCols;
  r = (index -c)/numCols;
  
  if(r < 0 || c < 0 || r >= numRows || c >= numCols)
    return false;
  else 
    return true;
}

bool CoverageGrid::getIndex(int &index, int r, int c)
{
  // Return the index corresponding to (r,c) if it is in the map
  if(!gridInit)
  {
    ROS_ERROR("Coverage Grid: Cannot get index before grid dimensions are set.");
    return false;
  }

  if(r < 0 || c < 0 || r >= numRows || c >= numCols)
  {
    return false;
  }
  else
  {
    index = r*numCols + c;
    if(index < 0 || index >= mapSize)
      return false;
    else
      return true;
  }
}

int CoverageGrid::getIndex(geometry_msgs::Pose pose)
{
  if(!isInitialized)
  {
    ROS_ERROR("Coverage Grid: Cannot get index of pose before cells are initialized.");
    return -1;
  }

  int r,c,i;
  r = 0;
  c = 0;
  i = 0;
  int lastR, lastC;
  lastR = -1;
  lastC = -1;
  geometry_msgs::Pose tempP = cells[i]->getPose();
  double distance = getDistance(tempP,pose);
  while(!(r == lastR && c == lastC) && r<getNumRows() && c<getNumCols())
  {
    lastC = c;
    lastR = r;
    if(getIndex(i,r+1,c))
    {
      geometry_msgs::Pose tempP = cells[i]->getPose();
      double tempD = getDistance(tempP,pose);
      if(tempD < distance)
      {
        distance = tempD;
        r++;
      }
    }
    if(getIndex(i,r,c+1))
    {
      geometry_msgs::Pose tempP = cells[i]->getPose();
      double tempD = getDistance(tempP,pose);
      if(tempD < distance)
      {
        distance = tempD;
        c++;
      }
    }
  }

  if(getIndex(i,r,c))
  {
    return i;
  }
  else
  {
    ROS_ERROR_STREAM("Coverage Grid : get index landed on invalid index. r: " << r << " c: " << c << " rows: " << getNumRows() << " cols: " << getNumCols());
    return -1;
  }
}

double CoverageGrid::getDistance(geometry_msgs::Pose &pose1, geometry_msgs::Pose &pose2)
{
  return sqrt(pow(pose1.position.x-pose2.position.x,2)+pow(pose1.position.y-pose2.position.y,2));
}

void CoverageGrid::getPositions(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds)
{
  inds.clear();
  int numFound = 0;
  for(int i = 0; i < cells.size() && numFound < poses.size(); i++)
  {
    bool contained = false;
    for(int j = 0; j < poses.size() && !contained; j++)
    {
      if(cells[i]->contains(poses[j]))
      {
        inds.push_back(i);
        numFound ++;
        contained = true;
      }
    }
  }
}

void CoverageGrid::getUnvisitedCells(std::vector<geometry_msgs::Pose> &poses)
{
  //std::cout << "Candidate unvisited cells: ";
  //for(int i = 0; i<poses.size(); i++)
  //{
  //  std::cout << poses[i].position.x << " " << poses[i].position.y << ", ";
  //}
  //std::cout << std::endl;

  std::vector<geometry_msgs::Pose> toCheck = poses;
  poses.clear();
  std::vector<int> positions;
  getPositions(toCheck, positions);

  //std::cout << "Candidate indices: ";
  for(int i = 0; i < positions.size(); i++)
  {
    //std::cout << positions[i] << ", ";
    if(!cells[positions[i]]->isVisited())
    {
      poses.push_back(cells[positions[i]]->getPose());
    }
  }
  //std::cout << std::endl;

  //std::cout << "Returning unvisited cells: ";
  //for(int i = 0; i<poses.size(); i++)
  //{
  //  std::cout << poses[i].position.x << " " << poses[i].position.y << ", ";
  //}
  //std::cout << std::endl;
}

void CoverageGrid::getUnvisitedCells(std::vector<geometry_msgs::Pose> poses, std::vector<int> &inds)
{
  //std::cout << "Candidate unvisited cells: ";
  //for(int i = 0; i<poses.size(); i++)
  //{
  //  std::cout << poses[i].position.x << " " << poses[i].position.y << ", ";
  //}
  //std::cout << std::endl;

  std::vector<geometry_msgs::Pose> toCheck = poses;
  poses.clear();
  std::vector<int> positions;
  getPositions(toCheck, positions);
  for(int i = 0; i < positions.size(); i++)
  {
    if(!cells[positions[i]]->isVisited())
    {
      inds.push_back(positions[i]);
    }
  }

  //std::cout << "Returning unvisited cells: ";
  //for(int i = 0; i<inds.size(); i++)
  //{
  //  std::cout << inds[i] << " " << inds[i] << ", ";
  //}
  //std::cout << std::endl;
}

int CoverageGrid::getSize()
{
  return mapSize;
}

int CoverageGrid::getNumCols()
{
  return numCols;
}

int CoverageGrid::getNumRows()
{
  return numRows;
}

bool CoverageGrid::getCell(int index, CoverageCell* &cell)
{
  if(index >= 0 && index < cells.size())
  {
    cell = cells[index];
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getCell was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::getNeighbors(int index, std::vector<CoverageCell*>& nbs)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->getNeighbors(nbs); 
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getNeighbors was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::getUnplannedNeighbors(int index, std::vector<CoverageCell*>& unbs)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->getUnplannedNeighbors(unbs);
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getUnplannedNeighbors was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::getValue(int index, CellValue &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->getValue();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getValue was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::setValue(int index, CellValue val)
{
  if(index >= 0 && index < cells.size())
  {
    return cells[index]->setValue(val);
  }
  else
  {
    ROS_ERROR("CoverageGrid::setValue was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::setPlanned(int index)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->setPlanned();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::setPlanned was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::visit(int index)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->visit();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::visit was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::visit(geometry_msgs::Pose pose)
{
  return visit(getIndex(pose));
}

bool CoverageGrid::setAttempted(int index)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->setAttempted();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::setAttempted was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::setAttempted(geometry_msgs::Pose pose)
{
  return setAttempted(getIndex(pose));
}

bool CoverageGrid::setReachable(int index, bool val)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->setReachable(val);
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::setReachable was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::isPlanned(int index, bool &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->isPlanned();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::isPlanned was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::isVisited(int index, bool &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->isVisited();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::isVisited was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::isAttempted(int index, bool &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->isAttempted();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::isAttempted was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::isReachable(int index, bool &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->isReachable();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::isReachable was passed invalid index.");
    return false;
  }
}

bool CoverageGrid::contains(int index, geometry_msgs::Pose pose, bool &val)
{
  if(index >= 0 && index < cells.size())
  {
    val = cells[index]->contains(pose);
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::contains was passed invalid index. Cannot check if pose is contained in cell.");
    return false;
  }
}

bool CoverageGrid::getPose(int index, geometry_msgs::Pose &pose)
{
  if(index >= 0 && index < cells.size())
  {
    pose = cells[index]->getPose();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getPose was passed invalid index. Cannot get cell's pose.");
    return false;
  }
}

bool CoverageGrid::getCorners(int index, std::vector<geometry_msgs::Pose> &poses)
{
  if(index >= 0 && index < cells.size())
  {
    poses = cells[index]->getCorners();
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::getCorners was passed invalid index. Cannot get cell corners.");
    return false;
  }
}

bool CoverageGrid::setCorners(int index, std::vector<geometry_msgs::Pose> verts)
{
  if(index >= 0 && index < cells.size())
  {
    cells[index]->setCorners(verts);
    return true;
  }
  else
  {
    ROS_ERROR("CoverageGrid::setCorners was passed invalid index. Cannot set cell corners.");
    return false;
  }
}

void CoverageGrid::print(std::string name)
{
  /********************************************************************************/
  /** This code block will print the wavefront. Use only for debugging purposes. **/
  /********************************************************************************/
  int grid_cols = getNumCols();
  int grid_rows = getNumRows();
  int index;

  // Convert occupancy grid to an image
  cv::Mat typeMat = cv::Mat(grid_rows, grid_cols, CV_8UC3);
  cv::Mat nbsMat = cv::Mat(grid_rows, grid_cols, CV_8UC3);
  cv::Mat reachMat = cv::Mat(grid_rows, grid_cols, CV_8UC3);
  cv::Mat visMat = cv::Mat(grid_rows, grid_cols, CV_8UC3);
  cv::Vec3b BLACK(0,0,0);
  cv::Vec3b GREEN(0,255,0);
  cv::Vec3b BLUE(0,0,255);
  cv::Vec3b RED(255,0,0);
  cv::Vec3b ORANGE(175,50,0);
  cv::Vec3b WHITE(255,255,255);
  cv::Vec3b GREY(100,100,100);

  for (int row = 0; row < grid_rows; row++)
  {
    for (int col = 0; col < grid_cols; col++)
    {
      index = row*grid_cols+col; // Finding the appropriate cell in vectorized form
      if (cells[index]->getValue() == OBSTRUCTED_CELL) // If the cell is an obstacle
        typeMat.at<cv::Vec3b>(row,col) = BLACK;
      else if (cells[index]->getValue() == OPEN_CELL) // If the cell is an obstacle
        typeMat.at<cv::Vec3b>(row,col) = WHITE;
      else if (cells[index]->getValue() == BORDER_CELL) // If the cell is an obstacle
        typeMat.at<cv::Vec3b>(row,col) = ORANGE;
      else
        typeMat.at<cv::Vec3b>(row,col) = GREY;

      std::vector<CoverageCell*> nbs;
      cells[index]->getNeighbors(nbs);
      int numNbs = nbs.size();
      switch(numNbs)
      {
        case 0:
          nbsMat.at<cv::Vec3b>(row,col) = BLACK;
          break;
        case 1:
          nbsMat.at<cv::Vec3b>(row,col) = RED;
          break;
        case 2:
          nbsMat.at<cv::Vec3b>(row,col) = ORANGE;
          break;
        case 3:
          nbsMat.at<cv::Vec3b>(row,col) = BLUE;
          break;
        case 4:
          nbsMat.at<cv::Vec3b>(row,col) = GREEN;
          break;
        default:
          nbsMat.at<cv::Vec3b>(row,col) = GREEN;
          break;
      }

      if(cells[index]->isReachable())
        reachMat.at<cv::Vec3b>(row,col) = GREEN;
      else
        reachMat.at<cv::Vec3b>(row,col) = BLACK;

      if(cells[index]->isVisited())
        visMat.at<cv::Vec3b>(row,col) = GREEN;
      else if(cells[index]->getValue() == OPEN_CELL)
        visMat.at<cv::Vec3b>(row,col) = BLUE;
      else if(cells[index]->getValue() == BORDER_CELL)
        visMat.at<cv::Vec3b>(row,col) = ORANGE;
      else if(cells[index]->getValue() == OBSTRUCTED_CELL)
        visMat.at<cv::Vec3b>(row,col) = BLACK;
      else
        visMat.at<cv::Vec3b>(row,col) = GREY;
    }
  }

  cv::cvtColor(typeMat, typeMat,  cv::COLOR_RGB2BGR);
  cv::cvtColor(nbsMat, nbsMat,  cv::COLOR_RGB2BGR);
  cv::cvtColor(reachMat, reachMat,  cv::COLOR_RGB2BGR);
  cv::cvtColor(visMat, visMat,  cv::COLOR_RGB2BGR);
  try
  {
    std::vector<int> comp_p;
    comp_p.push_back(cv::IMWRITE_PNG_COMPRESSION);
    comp_p.push_back(0);
    cv::imwrite("/home/avonster/" + name + "Types.bmp", typeMat);
    cv::imwrite("/home/avonster/" + name + "Nbs.bmp", nbsMat);
    cv::imwrite("/home/avonster/" + name + "Reach.bmp", reachMat);
    cv::imwrite("/home/avonster/" + name + "Vis.bmp", visMat,comp_p);
  }
  catch (std::runtime_error& ex)
  {
    std::cout << "Error converting image: " << ex.what() << std::endl;
  }
}

double CoverageGrid::getCellWidth()
{
  if(cells.size() < 1)
  {
    ROS_ERROR("Coverage grid: getCellWidth called with no cells.");
    return 0.0;
  }

  return cells[0]->getWidth();
}

} // namespace full_coverage

