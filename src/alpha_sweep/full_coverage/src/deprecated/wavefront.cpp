#include "wavefront.h"

namespace full_coverage
{

Wavefront::Wavefront() :
  startIndex(0),
  endIndex(0),
  isInitialized(false)
{
  // Do nothing
}

Wavefront::~Wavefront()
{
  // Do nothing
}

bool Wavefront::initialize(CellGrid map, int ot, int cd)
{
  // Initialize wave cells
  if(!waveCells.initialize(map, ot, 1))
  {
    ROS_ERROR("Wavefront: Wave cells failed to initalize");
    return false;
  }
  
  ROS_INFO("Wavefront: Wave cells initialized");
  
  // Initialize robot cells
  if(!robotCells.initialize(&waveCells, cd))
  {
    ROS_ERROR("Wavefront: Robot cells failed to initalize");
    return false;
  }
  
  ROS_INFO("Wavefront: Robot cells initalized");
  std::cout << "robot cells size: " << robotCells.getSize() << ". wavefront cells size: " << waveCells.getSize() << std::endl;
  
  // Set init flag
  isInitialized = true;
  
  return true;
}

bool Wavefront::fill(geometry_msgs::Pose startPose)
{
  // Set start state if it is valid
  if(!setStartIndex(startPose))
  {
    return false;
  }
  // Set start and end cell values
  waveCells.setValue(startIndex, WAVE_START);
  waveCells.setValue(endIndex, WAVE_END);
  
  // Fill remaining cells using wavefront propogation
  int fillCount = 2;      // Number of cells filled with wave numbers (not including obstructions)
  int waveNumber = 1;     // Number indicating the current wave
  int lastFillCount = 0;  // Number of cells filled with wave numbers in previous iteration
  while(fillCount != lastFillCount)
  {
    lastFillCount = fillCount;
    for(int i = 0; i<waveCells.getSize(); i++)
    {
      int val = 0;
      if(waveCells.getValue(i,val))
      {
        if(val == waveNumber-1)
        {
          fillCount += waveCells.setNeighbors(i, waveNumber);
        }
      }
      else
      {
        ROS_ERROR("Wavefront::fill waveCells going out of range.");
      }
    }
    waveNumber++;
  }
  
  // Update robot cells aggragate values now that wavefront cells have changed
  for(int i = 0; i<robotCells.getSize(); i++)
    robotCells.updateValue(i);
  
  std::cout << "printing wave cells" << std::endl;
  waveCells.print();
  std::cout << "printing robot cells" << std::endl;
  robotCells.print();
  
  return true;
}

bool Wavefront::getPlan(std::vector<geometry_msgs::Pose> &plan)
{
  // Use debugPoses vector to print out map of plan order
  std::vector<int> debugPoses;
  
  // Determine how many cells are to be in the full coverage plan
  int numCells = 0;     // number of poses we need to add to the plan
  for(int i = 0; i<waveCells.getSize(); i++)
  {
    // Cells to be planned do not include cells that the wavefront propogation could not reach, obstructions, or cells planned in a previous plan
    int tempVal = WAVE_INIT;
    bool tempPlan = true;
    if(waveCells.getValue(i,tempVal) && 
       waveCells.isPlanned(i,tempPlan) &&
       tempVal != WAVE_INIT && tempVal != WAVE_OBSTRUCTION && tempVal!= WAVE_BORDER && !tempPlan)
    {
      //std::cout << "planning cell " << i << ". wave value: " << tempVal << std::endl;
      numCells++;
    }
  }
  std::cout << "we will plan " << numCells << " cells." << std::endl;
  
  // Find the robot cell containing the start wavefront cell with the max wavefront value
  int rStartIndex = findRobotCellStart(startIndex);                    // index of the current cell being added to the plan
  int index = rStartIndex;
  if(index == -1)
    return false;
  
  // Plan order of poses
  int numPlanned = addPose(index-1, index, plan);   // Add the start pose as the first pose in the plan
  debugPoses.push_back(index);          // Make the start pose the first pose in debug_poses
  while(numPlanned < numCells)               // while we still have poses that need to be added to the plan
  {
    bool stuck = false;
    while(!stuck && numPlanned < numCells)  // if we get stuck, we exit the loop and use the hop function to find the closest unplanned cell
    {
      int newIndex = 0;
      if(robotCells.getMaxUnplannedNeighbor(index, newIndex))  // We can only move to a neighbor if it is valid and unplanned
      {
        int lastIndex = index;
        index = newIndex; // set the index to the neighbor we will move to
        numPlanned += addPose(lastIndex, index, plan);            // add the new cell to the plan
        debugPoses.push_back(index);
      }
      else // if all valid neighbors have been planned, we are stuck
      {
        //ROS_INFO("Stuck: no neighbors are unplanned.");
        stuck = true;
      }
      if(debugPoses.size()<5)
        printPlan(debugPoses);
    }
    
    if(numPlanned<numCells) // if we exited the previous loop because we were stuck, not because we were done planning
    {
      std::cout << "hopping after planning " << numPlanned << " cells." << std::endl;
      // When we get stuck, find the closest unplanned cell and go there
      int lastIndex = index;
      if(!robotCells.hop(index)) // if hop was unsuccessful, we should must not have any more cells to plan
      {
        std::cout << "failed to hop after planning " << numPlanned << " cells." << std::endl;
        numPlanned = numCells;
      }
      numPlanned += addPose(lastIndex, index, plan); // add new cell to the plan
      debugPoses.push_back(index);
    }
  }
  // return back to start cell
  addPose(index,rStartIndex,plan); 
  
  std::cout << "aborted after planning " << debugPoses.size() << " poses." << std::endl;
  printPlan(debugPoses);
  
  return true;
}

void Wavefront::printPlan(std::vector< int > poses)
{
  /**************************************************************************************************/
  /** This code block will print the order of the wavefront plan. Use only for debugging purposes. **/
  /**************************************************************************************************/
  int grid_cols = waveCells.getNumCols();
  int grid_rows = waveCells.getNumRows();
  
  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string default_col = "\033[0m";
  
  std::cout << "-----------------------------------------------------------------" << std::endl;
  int half_rw = floor(robotCells.getWidth()/2.0);
  int half_rh = floor(robotCells.getHeight()/2.0);
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      int index = 0;
      waveCells.getIndex(index, row, col); // Finding the appropriate cell in vectorized form
      int val = 0;
      if (waveCells.getValue(index,val) && val == WAVE_OBSTRUCTION) // If the cell is an obstacle
        std::cout<<color_red<< "|   |" <<default_col<<"|";
      else
      {
        int order = 0;
        int rIndex = 0;
        if(robotCells.getIndex(rIndex,row-half_rh,col-half_rw))
        {
          std::vector<int>::iterator it = std::find(poses.begin(),poses.end(),rIndex);
          if(it == poses.end())
            order = -1;
          else
            order = std::distance(poses.begin(), it);
        }
        else
          order = -1;
        
        if(order<0)
          if(val<0)
            std::cout<< "|  "<< val <<"|"; 
          else if(val<10)
            std::cout<< "|   "<< val <<"|"; 
          else if(val<100)
            std::cout<< "|  "<< val <<"|"; 
          else if(val<1000)
            std::cout<< "| "<< val <<"|"; 
          else
            std::cout<< "|"<< val <<"|"; 
        else if(order<10)
          std::cout<< "|   "<< color_red <<order << default_col <<"|"; 
        else if(order<100)
          std::cout<< "|  "<< color_red <<order << default_col <<"|"; 
        else if(order<1000)
          std::cout<< "| "<< color_red <<order << default_col <<"|"; 
        else
          std::cout<< "|"<< color_red <<order << default_col <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
}

bool Wavefront::setStartIndex(geometry_msgs::Pose startPose)
{
  // Find the cell that contains the start index
  int cellIndex = waveCells.getIndex(startPose);
  
  // Make sure the cell is valid and not obstructed
  if(cellIndex >= 0 && cellIndex < waveCells.getSize())
  {
    int val = 0;
    if(!waveCells.getValue(cellIndex, val) || WAVE_OBSTRUCTION == val)
    {
      ROS_ERROR("Wavefront: Start pose invalid: cell is occupied or not on map");
      return false;
    }
    startIndex = cellIndex;
  }
  else
  {
    ROS_ERROR("Wavefront: Start pose invalid: not within map bounds.");
    ROS_ERROR_STREAM("Start pose x: " << startPose.position.x << ". y: " << startPose.position.y);
    CoverageCell* temp1;
    CoverageCell* temp2;
    waveCells.getCell(0,temp1);
    waveCells.getCell(waveCells.getSize(), temp2);
    ROS_ERROR_STREAM("Wavefront: Map bounds x: " << temp1->getPose().position.x << " to " << temp2->getPose().position.x);
    ROS_ERROR_STREAM("Wavefront: Map bounds y: " << temp1->getPose().position.y << " to " << temp2->getPose().position.y);
    return false;
  }
  
  // Find a valid neighbor to place end index
  std::vector<CoverageCell*> startNeighbors;
  waveCells.getNeighbors(startIndex, startNeighbors);
  if(startNeighbors.size() > 0)
  {
    endIndex = startNeighbors[0]->getPosition();
    waveCells.reset(endIndex); // reset the end index. This is for replanning.. we need a place to end even if it has been planned.
    return true;
  }
  else
  {
    ROS_ERROR("Start pose invalid: has no valid neighbors");
    return false;
  }
  
  return true;
}

int Wavefront::findRobotCellStart(int startIndex)
{
  int i = 0;
  int r = 0;
  int c = 0;
  waveCells.getCoordinate(startIndex, r, c);
  std::vector<CoverageCell*> candidates;
  candidates.clear();
  for(int col = c; col>c-robotCells.getWidth(); col--)
  {
    for(int row = r; row>r-robotCells.getHeight(); row--)
    {
      if(robotCells.getIndex(i,row,col))
      {
        CoverageCell* cell;
        robotCells.getCell(i,cell);
        if(cell->getValue()>=0)
        {
          candidates.push_back(cell);
        }
      }
    }
  }
  
  if(candidates.size()<=0)
  {
    ROS_ERROR("Wavefront: No valid robot cell contains the start pose. Cannot get plan");
    return -1;
  }
  
  int maxI = 0;
  int maxVal = 0;
  for(int i = 0; i<candidates.size(); i++)
  {
    int temp = candidates[i]->getUnplannedValue();
    if(temp >= maxVal)
    {
      maxVal = temp;
      maxI = candidates[i]->getPosition();
    }
  }
  return maxI;
}

int Wavefront::addPose(int lastI, int i, std::vector<geometry_msgs::Pose>& plan)
{
  // Get the center pose of the cell to be added to the plan
  geometry_msgs::Pose pose;
  robotCells.getPose(i,pose);
  
  // Determine which direction the robot will be coming so we know what orientation it should stop in
  tf::Quaternion quat;
  if(i-lastI == 1) // the new cell is directly to the right of the old cell
  {
    quat.setRPY(0,0,0);
  }
  else if(i-lastI == -1) // the new cell is directly to left of the old cell
  {
    quat.setRPY(0,0,3.14);
  }
  else if(i-lastI < -1) // the new cell is far to the left or above the old cell
  {
    quat.setRPY(0,0,1.57);
  }
  else // the new cell is far to the right or below the old cell
  {
    quat.setRPY(0,0,-1.57);
  }
  tf::quaternionTFToMsg(quat,pose.orientation);
  
  // add the pose to the plan
  plan.push_back(pose);
  
  // tell the cell it has been planned
  int numPlanned = 0;
  robotCells.setPlanned(i, numPlanned);
  return numPlanned;
}

void Wavefront::resetCells(std::vector<geometry_msgs::Pose> cells)
{
  // Call reset on all cells in the cells vector
  for(int i = 0; i<cells.size(); i++)
  {
    int index = robotCells.getIndex(cells[i]);
    if(index >= 0 && index < robotCells.getSize()) // do not attempt to access a cell that does not exist
    {
      robotCells.reset(index); 
    }
  }
}

void Wavefront::reinit()
{
  // reinitialize wavefront so we can propogate a new wave. Note this does not change planned value, just the wave value
  for(int i = 0; i<robotCells.getSize(); i++)
  {
    robotCells.setValue(i,WAVE_INIT);
  }
}

void Wavefront::findEmptyNeighbor(geometry_msgs::Pose& pose)
{
  waveCells.findEmptyNeighbor(pose);
}

} // namespace full_coverage

