#include "rolling_map.h"
#include "omp.h"
#include <algorithm>
#include "ros/console.h"
//#include <chrono>

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
  minXP = x0;
  minYP = y0;
}

RollingMap::~RollingMap()
{
  // do nothing
}

void RollingMap::insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin)
{
  // do not insert while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  

#ifdef USE_CUDA
  CellMap occupied;
  int cloudSize = scancloud.size();
  int maxRay = getWidth() + (getHeight()/2.0) + 1; 
  float* fPoints = new float[cloudSize*3]; 
  int* iPoints= new int[cloudSize*3];
  float fStart[3] = {sensorOrigin.x, sensorOrigin.y, sensorOrigin.z};
  int iStart[3] = {0,0,0};
  float fStartVoxel[3] = {0,0,0};
  int* rayPoints = new int[cloudSize*maxRay*3];
  int* raySizes = new int[cloudSize];

  // Get index of sensor posistion
  if(!toIndex(fStart[0],fStart[1],fStart[2],iStart[0],iStart[1],iStart[2]))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because it is not contained in the map bounds. Map bounds: (" 
    << getMinXP() << ", " << getMinYP() << ", " << getMinZP() << ") to (" << getMaxXP() << ", " << getMaxYP() << ", " << getMaxZP() 
    << "). sensor position: (" << fStart[0] << ", " << fStart[1] << ", " << fStart[2] << ").");
    return; 
  }

  // Get position of start voxel
  if(!toPosition(iStart[0],iStart[1],iStart[2],fStartVoxel[0],fStartVoxel[1],fStartVoxel[2]))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because sensor voxel is not contained in the map bounds. Map bounds: (" 
    << getMinXI() << ", " << getMinYI() << ", " << getMinZI() << ") to (" << getMaxXI() << ", " << getMaxYI() << ", " << getMaxZI() 
    << "). sensor position: (" << iStart[0] << ", " << iStart[1] << ", " << iStart[2] << ").");
    return; 
  }

  for(int i = 0; i < scancloud.size(); i++)
  {
    // mark the point as occupied
    Coord temp;
    if(toIndex(scancloud[i].x, scancloud[i].y, scancloud[i].z,temp.x,temp.y,temp.z))
    {
      occupied.insert(temp);
    }

    // prepare points to be sent to cast rays function
    fPoints[i] = scancloud[i].x;
    fPoints[cloudSize+i] = scancloud[i].y;
    fPoints[2*cloudSize+i] = scancloud[i].z;
    iPoints[i] = temp.x;
    iPoints[cloudSize+i] = temp.y;
    iPoints[2*cloudSize+i] = temp.z;
  }

  // cast all rays on gpu for multithreading
  //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  if(!castRays(fPoints, iPoints, cloudSize, maxRay, fStart, iStart, fStartVoxel, rayPoints, raySizes, getMinXI(), getMaxXI(), getMinYI(), getMaxYI(), getMinZI(), getMaxZI(), resolution))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Error in cuda castRays function.");
    return;
  }

  //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
  //std::cout << "insertCloud took " << duration << " microseconds for " << scancloud.size() << " points." << std::endl;
  //t1 = std::chrono::high_resolution_clock::now(); 
 
  // update free cells
  setFree(cloudSize, maxRay, rayPoints, raySizes);

  // set occupied cells
  setOccupied(occupied);
  //t2 = std::chrono::high_resolution_clock::now();
  //duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
  //std::cout << "set free/occupied took " << duration << " microseconds." << std::endl;
  delete raySizes;
  delete rayPoints;
  delete iPoints;
  delete fPoints;
#else
  ros::Time start = ros::Time::now();
  //std::cout << "new cloud insert" << std::endl;
  CellMap free, occupied;
  #pragma omp parallel for schedule(guided)
  for(int i = 0; i < scancloud.size(); i++)
  {
    // mark the point as occupied
    //setPosition(scancloud[i].x, scancloud[i].y, scancloud[i].z, true);
    Coord temp;
    if(toIndex(scancloud[i].x, scancloud[i].y, scancloud[i].z,temp.x,temp.y,temp.z))
    #pragma omp critical (occupied_insert) 
    {
      occupied.insert(temp);
    }
 
    // cast ray and mark ray points as free
    castRay(scancloud[i], sensorOrigin, free);
  }

  //std::cout << "insertCloud took " << (ros::Time::now()-start).toSec() << " seconds." << std::endl;
  start = ros::Time::now();

  // update free cells
  setFree(free);
  setOccupied(occupied);
  //std::cout << "set free/occupied took " << (ros::Time::now()-start).toSec() << " seconds." << std::endl;
#endif
}

#ifdef USE_CUDA
void RollingMap::setFree(int cloudSize, int maxRay, int* rayPoints, int* raySizes)
{
  // write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  

  // iterate through free cells and erase them from map
  for(int i = 0; i < cloudSize; i++)
  {
    for(int j = 0; j < raySizes[i]-1; j++)
    {
      Coord coord = newCoord(rayPoints[i*maxRay+j],rayPoints[i*maxRay+(cloudSize*maxRay)+j],rayPoints[i*maxRay+(2*cloudSize*maxRay)+j]);
      map.erase(coord);
    }
  }
}
#else
void RollingMap::setFree(CellMap &free)
{
  // write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // iterate through free cells and erase them from map
  for(CellMap::iterator it = free.begin(); it != free.end(); ++it)
  {
    map.erase(*it);
  }
}
#endif
void RollingMap::setOccupied(CellMap &occupied)
{
  // write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};  

  // iterate through free cells and add them to the map
  for(CellMap::iterator it = occupied.begin(); it != occupied.end(); ++it)
  {
    map.insert(*it);
  }
}

#ifndef USE_CUDA
void RollingMap::castRay(const pcl::PointXYZ &occPoint, const pcl::PointXYZ &sensorOrigin, CellMap &free)
{
  // init vars
  bool done = false;

  float point[3] = {occPoint.x,occPoint.y,occPoint.z};
  float origin[3] = {sensorOrigin.x,sensorOrigin.y,sensorOrigin.z};

  // calculate normal vector in direction of sensor->point
  float direction[3] = {point[0]-origin[0],point[1]-origin[1],point[2]-origin[2]};
  float directionMagnitude = pow(pow(direction[0],2) + pow(direction[1],2) + pow(direction[2],2),0.5);

  // variables used for ray casting algorithm
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

  // check direction magnitude for divide by zero or same cell
  if(fabs(directionMagnitude) < resolution)
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray becuase magnitude of direction vector is very small");
    return;
  }

  // get the position of the cell that the sensor origin is in
  if(!toPosition(currentIndex[0],currentIndex[1],currentIndex[2],currentPosition[0],currentPosition[1],currentPosition[2]))
  {
    ROS_ERROR_THROTTLE(1.0, "RollingMap: Failed to get position of sensor origin cell. cannot raytrace.");
    return;
  }

  // set up initial values in each direction
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

  //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  // loop until we are out of map bounds
  while(!done)
  {
    // find direction of min error
    int dim = std::distance(accumulatedError, std::min_element(accumulatedError, accumulatedError + 3));

    // advance in direction of min error
    currentIndex[dim] += stepDirection[dim];
    accumulatedError[dim] += deltaError[dim]; 

    // done if we are at occ point
    if(usePI)
    {
      if(currentIndex[0] == pointIndex[0] &&
         currentIndex[1] == pointIndex[1] &&
         currentIndex[2] == pointIndex[2])
      {
        done = true;
      }
    }

    // if we are off the map, we are done. 
    if(!checkIndex(currentIndex[0],currentIndex[1],currentIndex[2]))
    {
      done = true;
    }
      
    //otherwise we mark the current index as unoccupied
    if(!done)
    #pragma omp critical (free_insert) 
    {
      free.insert(newCoord(currentIndex[0],currentIndex[1],currentIndex[2]));
    }
  }
  //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
  //if (duration > maxTime)
  //  maxTime = duration;
  //if (duration < minTime)
  //  minTime = duration;
}
#endif

void RollingMap::getMap(std::vector<pcl::PointXYZ> &mapcloud, int &minxi, int &minyi, float &minxp, float &minyp)
{
  // do not allow getMap while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  

  mapcloud.clear();

  minxi = getMinXI();
  minyi = getMinYI();
  minxp = getMinXP();
  minyp = getMinYP();

  // read lock map mutex
  boost::shared_lock<boost::shared_mutex> mlock{mapMutex};

  for(CellMap::iterator it = map.begin(); it != map.end(); it++)
  {
    pcl::PointXYZ point(it->x,it->y,it->z);
    mapcloud.push_back(point);
  }
}

void RollingMap::updatePosition(float x, float y)
{
  // write lock for translate mutex
  boost::unique_lock<boost::shared_mutex> tlock{translateMutex};

  ros::Time start = ros::Time::now();  

  // find indices of the new and old positions
  int xIndex, yIndex, zIndex, xOld, yOld, xChange, yChange;
  index(x,y,0.0,xIndex,yIndex,zIndex);
  index(getXPosition(),getYPosition(),0.0,xOld,yOld,zIndex);

  // calculate the shift from the old position to the new
  xChange = xIndex-xOld;
  yChange = yIndex-yOld;

  // clear cells that will be used for new space
  clearX(xChange);
  clearY(yChange);
  
  xPosition = x;
  yPosition = y;
  minXI += xChange;
  minYI += yChange;
  minXP += xChange*resolution;
  minYP += yChange*resolution;

  //std::cout << "translation took " << (ros::Time::now()-start).toSec() << " seconds" << std::endl;
}

void RollingMap::clearX(int shift)
{
  if(shift == 0)
    return;
  if(abs(shift) >= width)
  {
    clearAll();
    return;
  }

  // write lock map mutex
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

  // write lock map mutex
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
  // do not clear while we are translating the map
  boost::shared_lock<boost::shared_mutex> tlock{translateMutex};  
  
  // write lock map mutex
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

  // do not clear while we are translating the map
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

  // write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // delete elements in box
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

  // read lock map mutex
  boost::shared_lock<boost::shared_mutex> mlock{mapMutex};

  // lookup value
  CellMap::iterator it = map.find(coord);
  if(it != map.end())
    value = true;
  else
    value = false;
  return true;
}

bool RollingMap::setPosition(float x, float y, float z, bool value)
{
  // get the index, return false if out of bounds
  int xIndex, yIndex, zIndex;
  if(!toIndex(x,y,z,xIndex,yIndex,zIndex))
    return false;

  return setIndex(xIndex,yIndex,zIndex,value);
}

bool RollingMap::setIndex(int x, int y, int z, bool value)
{
  // return false if out of bounds
  if(!checkIndex(x,y,z))
    return false;

  Coord coord = newCoord(x,y,z);

  // write lock map mutex
  boost::unique_lock<boost::shared_mutex> mlock{mapMutex};

  // set value at adjusted indices
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
  // make sure index is in grid bounds
  if(!checkIndex(ix,iy,iz))
    return false;

  // convert index to position
  position(ix,iy,iz,px,py,pz);
  return true;
}

// returns true if position was in bounds of the map
bool RollingMap::toIndex(float px, float py, float pz, int &ix, int &iy, int &iz)
{
  // make sure position is in grid bounds
  if(!checkPosition(px,py,pz))
    return false;

  // convert position to index
  index(px,py,pz,ix,iy,iz);

  return true;
}

void RollingMap::position(int ix, int iy, int iz, float &px, float &py, float &pz)
{
  // index is allowed to be out of bounds.
  // we just want to know the x,y,z value in position space
  px = x0 + resolution/2.0 + ix*resolution;
  py = y0 + resolution/2.0 + iy*resolution;
  pz = z0 + resolution/2.0 + iz*resolution;
}

void RollingMap::index(float px, float py, float pz, int &ix, int &iy, int &iz)
{
  // position is allowed to be out of bounds.
  // we just want to know teh x,y,z value in index space
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
  return minXP;
}

float RollingMap::getMaxXP()
{
  return minXP + width*resolution;
}

float RollingMap::getMinYP()
{
  return minYP;
}

float RollingMap::getMaxYP()
{
  return minYP + width*resolution;  
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
