#pragma once

#include <coord.h>
#include <cuda_runtime.h>
#include <pcl/point_types.h>

namespace rolling_map{

class cudaVoxelGrid{
	public:
		__device__ cudaVoxelGrid(int width, int height, float res);

		__device__ ~cudaVoxelGrid();

		// __device__  operator=(const cudaVoxelGrid& cvg);

		__device__ rolling_map::Coord toIndex(const pcl::PointXYZ& p);

		__device__ void markVoxel(const rolling_map::Coord& c);

		__device__ void freeVoxel(const rolling_map::Coord& c);

		__device__ bool getVoxel(const rolling_map::Coord& c) const;

		__device__ bool offGrid(const rolling_map::Coord& c) const;

		uint8_t* voxels;

		const float resolution;
		const int width;
		const int height;

		float min_x;
		float min_y;
		float min_z;
};

} // end namespace rolling_map
