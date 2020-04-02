#ifndef _ROLLING_MAP_H_
#define _ROLLING_MAP_H_

#include <unordered_set>
#include <string>
#include <vector>
#include <atomic>
#include "boost/thread/shared_mutex.hpp"
#include "pcl_ros/point_cloud.h"

#ifdef USE_CUDA
bool castRays(float* fPoints, int* iPoints, int cloudSize, int maxRay, float* fStart, int* iStart, float* fStartVoxel, 
              int* outPoints, int* outSizes, int minX, int maxX, int minY, int maxY, int minZ, int maxZ, float resolution);
#endif

namespace rolling_map
{

//typedef std::atomic<bool>*** BoolArray;
struct Coord
{
public:
  int x;
  int y;
  int z;
};

struct HashCoord
{
public:
  size_t operator() (const Coord &coord) const 
  {
    return static_cast<size_t>(coord.x)
    + 1447*static_cast<size_t>(coord.y)
  + 345637*static_cast<size_t>(coord.z);
  }
};

struct CompareCoord
{
public:
  bool operator()(const Coord & c1, const Coord & c2) const
  {
    return (c1.x == c2.x) &&
           (c1.y == c2.y) &&
           (c1.z == c2.z);
  }
};

typedef std::unordered_set<Coord, HashCoord, CompareCoord> CellMap;

/**
* Rolling Array construct
*/
class RollingMap 
{
private:
  //BoolArray boolArray;    // Occupancy array whose size if fixed in constructor
  CellMap map;            // Unordered map of tracked occupied cells
  int width;              // Number of grid cells in x and y directions
  int height;             // Number of grid cells in z direction
  float resolution;      // Grid resoltuion (m/cell)
  float xPosition;       // Current x position of the robot in (m)
  float yPosition;       // Current y position of the robot in (m)
  float x0;
  float y0;
  float z0;              // Height of z[0] cells in array (m)
  float minXP;            // x position of lower left hand corner of grid
  float minYP;            // y position of lower left hand corner of grid
  int minXI;
  int minYI;
  boost::shared_mutex mapMutex;        // Mutex for thread safety when we translate the map
  boost::shared_mutex translateMutex;  // Mutex for thread safety when we translate the map

  void clearX(int shift);
  void clearY(int shift);
  void position(int ix, int iy, int iz, float &px, float &py, float &pz);
  void index(float px, float py, float pz, int &ix, int &iy, int &iz);
#ifndef USE_CUDA
  void castRay(const pcl::PointXYZ &occPoint, const pcl::PointXYZ &sensorOrigin, CellMap &free);
#endif
  Coord newCoord(int x, int y, int z);
  float getMinXP();
  float getMaxXP();
  float getMinYP();
  float getMaxYP();
  int getMinXI();
  int getMaxXI();
  int getMinYI();
  int getMaxYI();
  float getXPosition();
  float getYPosition();
#ifndef USE_CUDA
  void setFree(CellMap &free);
#else
  void setFree(int cloudSize, int maxRay, int* rayPoints, int* raySizes);
#endif
  void setOccupied(CellMap &free);

  /*
    Get occupancy value at (x,y,z) coord (meters)
  */
  bool atPosition(float x, float y, float z, bool &value);

  /*
    Get occupancy value at (x,y,z) index
  */
  bool atIndex(int x, int y, int z, bool &value);

  /*
    Set occupancy value at (x,y,z) coord (meters)
  */
  bool setPosition(float x, float y, float z, bool value);
  
  /*
    Set occupancy value at (x,y,z) index
  */
  bool setIndex(int x, int y, int z, bool value);

  /* 
    Check if (x,y,z) indices are in the bounds of the array
  */
  bool checkIndex(int x, int y, int z);

  /*
    Check if (x,y,z) position is in bounds of array
  */
  bool checkPosition(float x, float y, float z);

  /*
    Clear the area contained within the given x-y index polygon and z1-z2 coordinate heights
  */
  bool clearIndexBox(std::vector<std::vector<int>> polygon, int z1, int z2);
  bool pointInPoly(std::vector<int> point, std::vector<std::vector<int>> poly);

  /*
    Get the position of the cell at the supplied index
  */
  bool toPosition(int ix, int iy, int iz, float &px, float &py, float &pz);

  /*
    Get the index of the cell that contains the supplied position
  */
  bool toIndex(float px, float py, float pz, int &ix, int &iy, int &iz);

public:
  RollingMap(int w, int h, float res, float x, float y, float zmin);
  ~RollingMap();
  
  /////////////////////////////
  // Update functions
  /////////////////////////////
  /*
    Set the current position of the robot
  */
  void updatePosition(float x, float y);

  /////////////////////////////
  // Array access functions
  /////////////////////////////
  /*
    Clear the grid
  */
  void clearAll();

  /*
    Clear the area contained within the given x-y polygon and z1-z2 height
  */
  bool clearPositionBox(std::vector<std::vector<float>> polygon, float z1, float z2);

  /////////////////////////////
  //  Ray casting and point cloud insertion
  /////////////////////////////
  void insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin);
  void getMap(std::vector<pcl::PointXYZ> &mapcloud, int &minxi, int &minyi, float &minxp, float &minyp);

  /////////////////////////////
  // Getter functions
  /////////////////////////////
  int getWidth();
  int getHeight();
  float getResolution();
  float getMinZP();
  float getMaxZP();
  int getMinZI();
  int getMaxZI();

}; // RollingMap

} // namespace rolling_map

#endif // _ROLLING_MAP_H_
