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
  width(w),
  height(h),
  resolution(res),
  xPosition(x),
  yPosition(y),
  z0(zmin),
  minXI(0),
  minYI(0)
{
  #ifdef TIMEIT
  timer = std::make_unique<cpp_timer::Timer>();
  timer->allow_interruption = true;
  #endif

  // lock the mutex to prevent callbacks from starting early
  boost::unique_lock<boost::shared_mutex> tlock{translateMutex};  

  // set bound of grid
  if(width%2 == 0)
  {
    x0 = xPosition - (width/2)*resolution;
    y0 = yPosition - (width/2)*resolution;
  }
  else
  {
    x0 = xPosition - (width/2)*resolution - resolution/2.0;
    y0 = yPosition - (width/2)*resolution - resolution/2.0;
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

void RollingMap::insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin)
{
  // Do not insert while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  

#ifdef USE_CUDA
  CellMap occupied;
  int cloudSize = scancloud.size();
  float fStart[3] = {sensorOrigin.x, sensorOrigin.y, sensorOrigin.z};
  int iStart[3] = {0,0,0};
  pcl::PointXYZ start_voxel_loc;

  // Get index of sensor posistion
  if(!toIndex(fStart[0],fStart[1],fStart[2],iStart[0],iStart[1],iStart[2]))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because it is not contained in the map bounds. Map bounds: (" 
    << getMinXP() << ", " << getMinYP() << ", " << getMinZP() << ") to (" << getMaxXP() << ", " << getMaxYP() << ", " << getMaxZP() 
    << "). sensor position: (" << fStart[0] << ", " << fStart[1] << ", " << fStart[2] << ").");
    return; 
  }

  // Get position of start voxel
  if(!toPosition(iStart[0],iStart[1],iStart[2],start_voxel_loc.x,start_voxel_loc.y,start_voxel_loc.z))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because sensor voxel is not contained in the map bounds. Map bounds: (" 
    << getMinXI() << ", " << getMinYI() << ", " << getMinZI() << ") to (" << getMaxXI() << ", " << getMaxYI() << ", " << getMaxZI() 
    << "). sensor position: (" << iStart[0] << ", " << iStart[1] << ", " << iStart[2] << ").");
    return; 
  }

  // Cast all rays on gpu for multithreading
  TIC("CastRays")
  if(!castRays(scancloud, sensorOrigin, start_voxel_loc))
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Error in cuda castRays function.");
  TOC("CastRays")

#else

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
#endif
}

#ifdef USE_CUDA
void RollingMap::setFree(int cloudSize, int maxRay, int* rayPoints, int* raySizes)
{
  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  
  std::list<Coord> removeable_coords;

  // Iterate through free cells and erase them from map
  #pragma omp parallel for
  for(int i = 0; i < cloudSize; i++)
  {
    for(int j = 0; j < raySizes[i]-1; j++)
    {
      Coord coord = newCoord(rayPoints[i*maxRay+j],rayPoints[i*maxRay+(cloudSize*maxRay)+j],rayPoints[i*maxRay+(2*cloudSize*maxRay)+j]);
      if (map.count(coord))
      #pragma omp critical (record_coord)
      {
        removeable_coords.push_back(coord);
      }
    }
  }

  for (const Coord& c : removeable_coords){
    map.erase(c);
  }
}

#else
void RollingMap::setFree(CellMap &free)
{
  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // Iterate through free cells and erase them from map
  for(CellMap::iterator it = free.begin(); it != free.end(); ++it)
  {
    map.erase(*it);
  }
}
#endif
void RollingMap::setOccupied(CellMap &occupied)
{
  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  

  // Iterate through free cells and add them to the map
  for(CellMap::iterator it = occupied.begin(); it != occupied.end(); ++it)
  {
    map.insert(*it);
  }
}

// Non-CUDA castRay method inspired by ROS octomap: http://wiki.ros.org/octomap
#ifndef USE_CUDA
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
  if(fabs(directionMagnitude) < resolution)
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

    float voxelBorder = currentPosition[dir] + stepDirection[dir]*resolution*0.5;
    accumulatedError[dir] = (voxelBorder - origin[dir])/direction[dir];
    deltaError[dir] = resolution/fabs(direction[dir]);
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
      free.insert(newCoord(currentIndex[0],currentIndex[1],currentIndex[2]));
    }
  }
}
#endif

std::vector<pcl::PointXYZ> RollingMap::getMap()
{
  std::vector<pcl::PointXYZ> mapcloud;

  // Read lock map mutex
  boost::shared_lock<boost::shared_mutex> mlock{mapMutex};

  // Do not allow getMap while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex}; 

#ifdef USE_CUDA

  const size_t num_bits = width*width*height;
  const size_t num_ints = num_bits/voxel_block_size + 1;
  std::vector<voxel_block_t> voxel_grid(num_ints);
  CUDA_SAFE(cudaMemcpy(voxel_grid.data(), d_voxel_data_, num_ints*sizeof(uint32_t), cudaMemcpyDeviceToHost));

  for (size_t bit_idx = 0; bit_idx < num_bits; bit_idx++){
    const size_t  byte_idx = bit_idx/voxel_block_size;
    const voxel_block_t bit_mask = static_cast<voxel_block_t>(1) << (bit_idx % voxel_block_size);

    if (voxel_grid[byte_idx] & bit_mask){
      mapcloud.emplace_back(
        (bit_idx % width),
        (bit_idx / width) % width,
        (bit_idx / width / width) % height
      );
    }
  }

#else

  for(CellMap::iterator it = map.begin(); it != map.end(); it++)
  {
    mapcloud.emplace_back(it->x,it->y,it->z);
  }

#endif

  return mapcloud;
}

#ifndef USE_CUDA
void RollingMap::updatePosition(float x, float y)
{
  // Write lock for translate mutex
  boost::unique_lock<boost::shared_mutex> tlock{translateMutex};

  ros::Time start = ros::Time::now();  

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
  
  xPosition = x;
  yPosition = y;
  minXI += xChange;
  minYI += yChange;
  x0 += xChange*resolution;
  y0 += yChange*resolution;
}
#endif

void RollingMap::clearX(int shift)
{
  if(shift == 0)
    return;
  if(abs(shift) >= width)
  {
    clearAll();
    return;
  }

  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  

  int clip;
  if(shift > 0)
    clip = getMinXI() + shift;
  else
    clip = getMaxXI() + shift;

  for(CellMap::iterator it = map.begin(); it != map.end();)
  {
    if((shift > 0 && it->x < clip) || (shift < 0 && it->x > clip))
      it = map.erase(it);
    else
      it++;
  }
}

void RollingMap::clearY(int shift)
{
  if(shift == 0)
    return;
  if(abs(shift) >= width)
  {
    clearAll();
    return;
  }

  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  

  int clip;
  if(shift > 0)
    clip = getMinYI() + shift;
  else
    clip = getMaxYI() + shift;

  for(CellMap::iterator it = map.begin(); it != map.end();)
  {
    if((shift > 0 && it->y < clip) || (shift < 0 && it->y > clip))
      it = map.erase(it);
    else
      it++;
  }
}

void RollingMap::clearAll()
{
  // Do not clear while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  
  
  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  map.clear();
}

bool RollingMap::clearPositionBox(std::vector<std::vector<float>> polygon, float z1, float z2)
{
  if(polygon.size() <= 2)
  {
    ROS_ERROR("RollingMap: clearPositionBox polygon size must be at least 2! Cannot clear polygon.");
    return false;
  }

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

  // Do not clear while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  

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

  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // Delete elements in box
  for(CellMap::iterator it = map.begin(); it != map.end();)
  {
    std::vector<int> point;
    point.push_back(it->x);
    point.push_back(it->y);
    if(pointInPoly(point,polygon) &&
       it->z >= z1 && it->z <= z2)
    {
      it = map.erase(it);
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

bool RollingMap::atPosition(float x, float y, float z, bool &value)
{
  // Convert position values to indices (with position checking)
  int xIndex, yIndex, zIndex;
  if(!toIndex(x,y,z,xIndex,yIndex,zIndex))
    return false;

  return atIndex(xIndex, yIndex, zIndex, value);
}

bool RollingMap::atIndex(int x, int y, int z, bool &value)
{
  // Make sure indices are in map bounds
  if(!checkIndex(x,y,z))
    return false;

  Coord coord = newCoord(x,y,z);

  // Read lock map mutex
  boost::shared_lock<boost::shared_mutex> mlock{mapMutex};

  // Lookup value
  CellMap::iterator it = map.find(coord);
  if(it != map.end())
    value = true;
  else
    value = false;
  return true;
}

bool RollingMap::setPosition(float x, float y, float z, bool value)
{
  // Get the index, return false if out of bounds
  int xIndex, yIndex, zIndex;
  if(!toIndex(x,y,z,xIndex,yIndex,zIndex))
    return false;

  return setIndex(xIndex,yIndex,zIndex,value);
}

bool RollingMap::setIndex(int x, int y, int z, bool value)
{
  // Return false if out of bounds
  if(!checkIndex(x,y,z))
    return false;

  Coord coord = newCoord(x,y,z);

  // Write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // Set value at adjusted indices
  if(value)
    map.insert(coord);
  else
    map.erase(coord);

  return true;
}

bool RollingMap::checkIndex(int x, int y, int z)
{
  if(x >= getMinXI() && x <= getMaxXI() && 
     y >= getMinYI() && y <= getMaxYI() && 
     z >= getMinZI() && z <= getMaxZI())
  {
    return true;
  }
  else
    return false;
}

bool RollingMap::checkPosition(float x, float y, float z)
{
  if(x >= getMinXP() && x < getMaxXP() && 
     y >= getMinYP() && y < getMaxYP() && 
     z >= getMinZP() && z < getMaxZP())
  {
    return true;
  }
  else
    return false;
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
  px = x0 + resolution/2.0 + ix*resolution;
  py = y0 + resolution/2.0 + iy*resolution;
  pz = z0 + resolution/2.0 + iz*resolution;
}

void RollingMap::index(float px, float py, float pz, int &ix, int &iy, int &iz)
{
  // Position is allowed to be out of bounds.
  // We just want to know teh x,y,z value in index space
  ix = (px - x0)/resolution;
  iy = (py - y0)/resolution;
  iz = (pz - z0)/resolution;
}

int RollingMap::getWidth()
{
  return width;
}

int RollingMap::getHeight()
{
  return height;
}

float RollingMap::getResolution()
{
  return resolution;
}

float RollingMap::getXPosition()
{
  return xPosition;
}

float RollingMap::getYPosition()
{
  return yPosition;
}

float RollingMap::getMinXP()
{
  return x0;
}

float RollingMap::getMaxXP()
{
  return x0 + width*resolution;
}

float RollingMap::getMinYP()
{
  return y0;
}

float RollingMap::getMaxYP()
{
  return y0 + width*resolution;  
}

float RollingMap::getMinZP()
{
  return z0;
}

float RollingMap::getMaxZP()
{
  return z0 + height*resolution;
}

int RollingMap::getMinXI()
{
  return minXI;
}

int RollingMap::getMaxXI()
{
  return minXI + (width-1);
}

int RollingMap::getMinYI()
{
  return minYI;
}

int RollingMap::getMaxYI()
{
  return minYI + (width-1);  
}

int RollingMap::getMinZI()
{
  return 0;
}

int RollingMap::getMaxZI()
{
  return height-1;
}

Coord RollingMap::newCoord(int x, int y, int z)
{
  Coord coord;
  coord.x = x;
  coord.y = y;
  coord.z = z;
  return coord;
}

} // namespace rolling_map
