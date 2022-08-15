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
*********************************************************************/

#include <algorithm>
#include "omp.h"
#include "ros/console.h"
#include "cuda_safe.cuh"
#include "rolling_map.h"

#ifdef USE_CUDA
#include <cuda_runtime.h>
#include "cuda_voxel_grid.cuh"
#endif

#ifdef TIMEIT
#define TIMER_INSTANCE timer
#define TIC(x) timer->tic(x);
#define TOC(x) timer->toc(x);
#define ATIC TIMER_PTIC;
#define ATOC TIMER_PTOC;
#else
#define TIC(x)
#define TOC(x)
#define ATIC
#define ATOC
#endif

namespace rolling_map
{

RollingMap::RollingMap(int w, int h, float res, float x, float y, float zmin) :
  width_(w),
  height_(h),
  resolution_(res),
  xPosition_(x),
  yPosition_(y),
  z0_(zmin)
{
  #ifdef TIMEIT
  timer = std::make_unique<cpp_timer::Timer>();
  timer->allow_interruption = true;
  #endif

  // lock the mutex to prevent callbacks from starting early
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);  

  // set bound of grid
  if(width_%2 == 0)
  {
    x0_ = xPosition_ - (width_/2)*resolution_;
    y0_ = yPosition_ - (width_/2)*resolution_;
  }
  else
  {
    x0_ = xPosition_ - (width_/2)*resolution_ - resolution_/2.0;
    y0_ = yPosition_ - (width_/2)*resolution_ - resolution_/2.0;
  }

  CUDA_ONLY(
    if(cudaInit()) 
      ROS_INFO("Voxel grid succesfully allocated on the GPU");
    else 
      ROS_ERROR("Voxel grid initialization failed!");
  )
}

RollingMap::~RollingMap()
{
  #ifdef TIMEIT
  timer->summary(cpp_timer::Timer::BY_AVERAGE);
  #endif

  CUDA_ONLY(cudaFree(d_voxel_grid_));
}

#ifndef USE_CUDA
void RollingMap::insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin)
{
  // Do not insert while we are translating the map
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);  

  CellMap free, occupied;
  #pragma omp parallel for schedule(guided)
  for(int i = 0; i < scancloud.size(); i++)
  {
    // Mark the point as occupied
    Coord temp;
    if(toIndex(scancloud[i].x, scancloud[i].y, scancloud[i].z,temp.x,temp.y,temp.z))
    #pragma omp critical (occupied_insert) 
    {
      occupied.insert(temp);
    }
 
    // Cast ray and mark ray points as free
    castRay(scancloud[i], sensorOrigin, free);
  }

  // Update free cells
  setFree(free);
  setOccupied(occupied);
}

void RollingMap::setFree(CellMap &free)
{
  // Iterate through free cells and erase them from map
  for(CellMap::iterator it = free.begin(); it != free.end(); ++it)
  {
    map_.erase(*it);
  }
}

void RollingMap::setOccupied(CellMap &occupied)
{
  // Iterate through free cells and add them to the map
  for(CellMap::iterator it = occupied.begin(); it != occupied.end(); ++it)
  {
    map_.insert(*it);
  }
}

// Non-CUDA castRay method inspired by ROS octomap: http://wiki.ros.org/octomap
void RollingMap::castRay(const pcl::PointXYZ &occPoint, const pcl::PointXYZ &sensorOrigin, CellMap &free)
{
  // Init vars
  bool done = false;

  float point[3] = {occPoint.x,occPoint.y,occPoint.z};
  float origin[3] = {sensorOrigin.x,sensorOrigin.y,sensorOrigin.z};

  // Calculate normal vector in direction of sensor->point
  float direction[3] = {point[0]-origin[0],point[1]-origin[1],point[2]-origin[2]};
  float directionMagnitude = pow(pow(direction[0],2) + pow(direction[1],2) + pow(direction[2],2),0.5);

  // Variables used for ray casting algorithm
  int stepDirection[3];        // +/- step in each cardinal direction
  float accumulatedError[3];  // error accumulated in each direction
  float deltaError[3];        // change in error accumulated for a step in a direction
  int currentIndex[3];         // for tracking the index as we trace
  float currentPosition[3];   // x,y,z position of current index
  int pointIndex[3];           // index of final occupied point
  bool usePI = false;          // we only check for the final point if it is on the map, 
                               // otherwise we are done when we leave the map

  // Get index of sensor posistion
  if(!toIndex(origin[0],origin[1],origin[2],currentIndex[0],currentIndex[1],currentIndex[2]))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because it is not contained in the map bounds. Map bounds: (" << getMinXP() << ", " << getMinYP() << ", " << getMinZP() << ") to (" << getMaxXP() << ", " << getMaxYP() << ", " << getMaxZP() << "). sensor position: (" << origin[0] << ", " << origin[1] << ", " << origin[2] << ").");
    return; 
  }

  // If the occupied point is in the map, we get its index and use it as a stopping point
  if(toIndex(point[0],point[1],point[2],pointIndex[0],pointIndex[1],pointIndex[2]))
    usePI = true;

  // Check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < resolution_)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray becuase magnitude of direction vector is very small");
    return;
  }

  // Get the position of the cell that the sensor origin is in
  if(!toPosition(currentIndex[0],currentIndex[1],currentIndex[2],currentPosition[0],currentPosition[1],currentPosition[2]))
  {
    ROS_ERROR_THROTTLE(1.0, "RollingMap: Failed to get position of sensor origin cell. cannot raytrace.");
    return;
  }

  // Set up initial values in each direction
  for(int dir = 0; dir < 3; dir++)
  {
    direction[dir] /= directionMagnitude;
    if(direction[dir] > 0.0)
      stepDirection[dir] = 1;
    else if(direction[dir] < 0.0)
      stepDirection[dir] = -1;

    float voxelBorder = currentPosition[dir] + stepDirection[dir]*resolution_*0.5;
    accumulatedError[dir] = (voxelBorder - origin[dir])/direction[dir];
    deltaError[dir] = resolution_/fabs(direction[dir]);
  }

  // Loop until we are out of map bounds
  while(!done)
  {
    // Find direction of min error
    int dim = std::distance(accumulatedError, std::min_element(accumulatedError, accumulatedError + 3));

    // Advance in direction of min error
    currentIndex[dim] += stepDirection[dim];
    accumulatedError[dim] += deltaError[dim]; 

    // Done if we are at occ point
    if(usePI)
    {
      if(currentIndex[0] == pointIndex[0] &&
         currentIndex[1] == pointIndex[1] &&
         currentIndex[2] == pointIndex[2])
      {
        done = true;
      }
    }

    // If we are off the map, we are done. 
    if(!checkIndex(currentIndex[0],currentIndex[1],currentIndex[2]))
    {
      done = true;
    }
      
    // Otherwise we mark the current index as unoccupied
    if(!done)
    #pragma omp critical (free_insert) 
    {
      free.emplace(currentIndex[0],currentIndex[1],currentIndex[2]);
    }
  }
}

std::vector<pcl::PointXYZ> RollingMap::getMap()
{
  std::vector<pcl::PointXYZ> mapcloud;

  // Read lock map mutex
  std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

  for(CellMap::iterator it = map_.begin(); it != map_.end(); it++)
  {
    mapcloud.emplace_back(it->x,it->y,it->z);
  }


  return mapcloud;
}

void RollingMap::updatePosition(float x, float y)
{
  // Write lock for map mutex
  std::unique_lock<std::shared_mutex> write_lock{map_mutex_};

  // Find indices of the new and old positions
  int xIndex, yIndex, zIndex, xOld, yOld, xChange, yChange;
  index(x,y,0.0,xIndex,yIndex,zIndex);
  index(getXPosition(),getYPosition(),0.0,xOld,yOld,zIndex);

  // Calculate the shift from the old position to the new
  xChange = xIndex-xOld;
  yChange = yIndex-yOld;

  // Clear cells that will be used for new space
  clearX(xChange);
  clearY(yChange);
  
  xPosition_ = x;
  yPosition_ = y;
  x0_ += xChange*resolution_;
  y0_ += yChange*resolution_;
}

void RollingMap::clearX(int shift)
{
  if(shift == 0)
    return;
  if(abs(shift) >= width_)
  {
    clearAll();
    return;
  } 

  int clip;
  if(shift > 0)
    clip = shift;
  else
    clip = getMaxXI() + shift;

  for(CellMap::iterator it = map_.begin(); it != map_.end();)
  {
    if((shift > 0 && it->x < clip) || (shift < 0 && it->x > clip))
      it = map_.erase(it);
    else
      it++;
  }
}

void RollingMap::clearY(int shift)
{
  if(shift == 0)
    return;
  if(abs(shift) >= width_)
  {
    clearAll();
    return;
  } 

  int clip;
  if(shift > 0)
    clip = shift;
  else
    clip = getMaxYI() + shift;

  for(CellMap::iterator it = map_.begin(); it != map_.end();)
  {
    if((shift > 0 && it->y < clip) || (shift < 0 && it->y > clip))
      it = map_.erase(it);
    else
      it++;
  }
}

void RollingMap::clearAll()
{
  // Write lock map mutex
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

  map_.clear();
}
#endif

bool RollingMap::clearPositionBox(std::vector<std::vector<float>> polygon, float z1, float z2)
{
  if(polygon.size() <= 2)
  {
    ROS_ERROR("RollingMap: clearPositionBox polygon size must be at least 2! Cannot clear polygon.");
    return false;
  }

  // Write lock map mutex
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

  ROS_INFO_STREAM("RollingMap: Clearing position box. z1" << z1 << " z2: " << z2);
  std::vector<std::vector<int>> ipoly;
  int iz1;
  for(int i = 0; i < polygon.size(); i++)
  {
    if(polygon[i].size() !=2)
    {
      ROS_ERROR("RollingMap: clearPositionBox polygon should contain vectors of size 2 (x and y). Cannot clear polygon.");
      return false;
    }
    int x,y;
    index(polygon[i][0],polygon[i][1],z1,x,y,iz1);
    std::vector<int> point;
    point.push_back(x);
    point.push_back(y);
    ipoly.push_back(point);
    ROS_INFO_STREAM("RollingMap: point in poly: (" << x << ", " << y << ")");
  }
  int ix, iy, iz2;
  index(polygon[0][0],polygon[0][1],z2,ix,iy,iz2);

  return clearIndexBox(ipoly, iz1, iz2);
}

bool RollingMap::clearIndexBox(std::vector<std::vector<int>> polygon, int z1, int z2)
{
  if(polygon.size() <= 2)
  {
    ROS_ERROR("RollingMap: clearIndexBox polygon size must be at least 2! Cannot clear polygon.");
    return false;
  }

  if(z1>z2)
  {
    std::swap(z1,z2);
  }
  if(z1==z2)
    return false;

  ROS_INFO_STREAM("RollingMap: Clearing index box. z1" << z1 << " z2: " << z2);

  // Delete elements in box
  for(CellMap::iterator it = map_.begin(); it != map_.end();)
  {
    std::vector<int> point;
    point.push_back(it->x);
    point.push_back(it->y);
    if(pointInPoly(point,polygon) &&
       it->z >= z1 && it->z <= z2)
    {
      it = map_.erase(it);
    }
    else
      it++;
  }
  return true;
}

bool RollingMap::pointInPoly(std::vector<int> point, std::vector<std::vector<int>> poly)
{
  if(point.size() != 2)
  {
    ROS_ERROR("RollingMap: pointInPoly point should be size 2");
    return false;
  }
  int pos = 0;
  int neg = 0;
  for(int i = 0; i < poly.size(); i++)
  {
    if(poly[i].size() != 2)
    {
      ROS_ERROR("RollingMap: pointInPoly polygon should contain vectors of size 2 (x and y). Cannot check point.");
      return false;
    }

    int i2 = i + 1;
    if(i2 >= poly.size())
      i2 = 0;

    int cross = (point[0] - poly[i][0])*(poly[i2][1] - poly[i][1]) - (point[1] - poly[i][1])*(poly[i2][0] - poly[i][0]); 
    if(cross > 0)
      pos++;
    else if (cross < 0)
      neg++;

    if(pos > 0 && neg > 0)
      return false;
  }
  return true;
}

#ifndef USE_CUDA
bool RollingMap::atPosition(float x, float y, float z, bool &value)
{
  // Read lock map mutex
  std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

  // Convert position values to indices (with position checking)
  int xIndex, yIndex, zIndex;
  if(!toIndex(x,y,z,xIndex,yIndex,zIndex))
    return false;

  return atIndex(xIndex, yIndex, zIndex, value);
}

bool RollingMap::atIndex(int x, int y, int z, bool &value)
{
  // Read lock map mutex
  std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

  // Make sure indices are in map bounds
  if(!checkIndex(x,y,z))
    return false;

  Coord coord(x,y,z);

  // Lookup value
  CellMap::iterator it = map_.find(coord);
  value = (it != map_.end());
  return true;
}

bool RollingMap::setPosition(float x, float y, float z, bool value)
{
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

  // Get the index, return false if out of bounds
  int xIndex, yIndex, zIndex;
  if(!toIndex(x,y,z,xIndex,yIndex,zIndex))
    return false;

  Coord coord(x,y,z);

  // Set value at adjusted indices
  if(value)
    map_.insert(coord);
  else
    map_.erase(coord);

  return true;
}

bool RollingMap::setIndex(int x, int y, int z, bool value)
{
  // Write lock map mutex
  std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);

  // Return false if out of bounds
  if(!checkIndex(x,y,z))
    return false;

  Coord coord(x,y,z);

  // Set value at adjusted indices
  if(value)
    map_.insert(coord);
  else
    map_.erase(coord);

  return true;
}
#endif

bool RollingMap::checkIndex(int x, int y, int z)
{
  return(x >= 0 && x <= getMaxXI() && 
         y >= 0 && y <= getMaxYI() && 
         z >= 0 && z <= getMaxZI());
}

bool RollingMap::checkPosition(float x, float y, float z)
{
  return(x >= getMinXP() && x < getMaxXP() && 
         y >= getMinYP() && y < getMaxYP() && 
         z >= getMinZP() && z < getMaxZP());
}

bool RollingMap::toPosition(int ix, int iy, int iz, float &px, float &py, float &pz)
{
  // Make sure index is in grid bounds
  if(!checkIndex(ix,iy,iz))
    return false;

  // Convert index to position
  position(ix,iy,iz,px,py,pz);
  return true;
}

// Returns true if position was in bounds of the map
bool RollingMap::toIndex(float px, float py, float pz, int &ix, int &iy, int &iz)
{
  // Make sure position is in grid bounds
  if(!checkPosition(px,py,pz))
    return false;

  // Convert position to index
  index(px,py,pz,ix,iy,iz);

  return true;
}

void RollingMap::position(int ix, int iy, int iz, float &px, float &py, float &pz)
{
  // Index is allowed to be out of bounds.
  // We just want to know the x,y,z value in position space
  px = x0_ + resolution_/2.0 + ix*resolution_;
  py = y0_ + resolution_/2.0 + iy*resolution_;
  pz = z0_ + resolution_/2.0 + iz*resolution_;
}

void RollingMap::index(float px, float py, float pz, int &ix, int &iy, int &iz)
{
  // Position is allowed to be out of bounds.
  // We just want to know the x,y,z value in index space
  ix = (px - x0_)/resolution_;
  iy = (py - y0_)/resolution_;
  iz = (pz - z0_)/resolution_;
}

int RollingMap::getWidth()
{
  return width_;
}

int RollingMap::getHeight()
{
  return height_;
}

float RollingMap::getResolution()
{
  return resolution_;
}

float RollingMap::getXPosition()
{
  return xPosition_;
}

float RollingMap::getYPosition()
{
  return yPosition_;
}

float RollingMap::getMinXP()
{
  return x0_;
}

float RollingMap::getMaxXP()
{
  return x0_ + width_*resolution_;
}

float RollingMap::getMinYP()
{
  return y0_;
}

float RollingMap::getMaxYP()
{
  return y0_ + width_*resolution_;  
}

float RollingMap::getMinZP()
{
  return z0_;
}

float RollingMap::getMaxZP()
{
  return z0_ + height_*resolution_;
}

int RollingMap::getMaxXI()
{
  return width_-1;
}

int RollingMap::getMaxYI()
{
  return width_-1;  
}

int RollingMap::getMaxZI()
{
  return height_-1;
}

} // namespace rolling_map
