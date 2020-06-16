#ifndef TOPO_MAP_H
#define TOPO_MAP_H


#include <math.h>         					// general mathy stuff
#include <ros/ros.h>						// all of ros
#include <ros/package.h>					// how I find the directory path
#include <algorithm>						// for finding things in vectors
#include <iostream>							// I/O file stuff
#include <fstream>							// I/O file stuff
#include <sstream>							// I/O file stuff

namespace topo_map
{

struct Node {

}

struct Edge {
	
}

class TopoMap
{
public:

  /*
   * Constructor
   * @param _n - reference to the node handle
   */
  TopoMap(ros::NodeHandle& _n);

  /*
   * Destructor
   */
  ~TopoMap();

private:

  // Node handle
  ros::NodeHandle& node_;

  // Subscriber for different map processing requests
  ros::ServiceServer process_node_srv;

  // Oft used throughout member variables
  std::string dir_path_;  // path to the project directory found on launch
  int resolution_;  // of the map (usually around 20)


  };
} // end namespace topo_map

#endif