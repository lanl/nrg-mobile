#include "rolling_map.h"
#include "cuda_voxel_grid.cuh"
#include <cuda_runtime.h>

#define THREADS_PER_BLOCK 1024
#define X_CHANGE 1
#define Y_CHANGE 2

__global__ void translateMap(rolling_map::cudaVoxelGrid* voxel_grid, int change, int dir){

	// index of thread in either the x or y directions
	size_t long_index = threadIdx.x + blockIdx.x * blockDim.x;

	// Index of thread in the z direction
	size_t z_index = threadIdx.z + blockIdx.z * blockDim.z;

	switch(dir){
		case X_CHANGE:
			// Determine direcion
			for (int x = 0; x < voxel_grid->width; x++){

			}
			break;
	}

}

void rolling_map::RollingMap::updatePosition(float x, float y){

  // find indices of the new and old positions
  int xIndex, yIndex, zIndex, xOld, yOld;
  index(x,y,0.0,xIndex,yIndex,zIndex);
  index(getXPosition(),getYPosition(),0.0,xOld,yOld,zIndex);

  // calculate the shift from the old position to the new
  int xChange = xIndex-xOld;
  int yChange = yIndex-yOld;

  dim3 block_size;
  block_size.x = THREADS_PER_BLOCK/8;
  block_size.y = 1;
  block_size.z = 8;

  dim3 grid_size;

  translateMap<<<1,1>>>(d_voxel_grid_, yChange, Y_CHANGE);
  cudaDeviceSynchronize();
  translateMap<<<1,1>>>(d_voxel_grid_, xChange, X_CHANGE);
  cudaDeviceSynchronize();
  
  xPosition = x;
  yPosition = y;
  minXI += xChange;
  minYI += yChange;
  minXP += xChange*resolution;
  minYP += yChange*resolution;

}