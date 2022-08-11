#pragma once

#include <coord.h>
#include <cuda_runtime.h>
#include <pcl/point_types.h>

namespace rolling_map{

typedef uint32_t voxel_block_t;
#define voxel_block_size (sizeof(voxel_block_t)*8)

struct voxelIndex{
	voxel_block_t* mem_ptr;
	voxel_block_t  bit_mask;
};

class cudaVoxelGrid{
	public:
		__device__ cudaVoxelGrid(int width, int height, float res);

		__device__ ~cudaVoxelGrid();

		__device__ Coord toIndex(const pcl::PointXYZ& p) const;

		__device__ voxelIndex getVoxelMask(const Coord& c) const;

		__device__ void markVoxel(const Coord& c);

		__device__ void freeVoxel(const Coord& c);

		__device__ bool getVoxel(const Coord& c) const;

		__device__ bool offGrid(const Coord& c) const;

		voxel_block_t* voxels;

		const float resolution;
		const int width;
		const int height;

		float min_x;
		float min_y;
		float min_z;
};

} // end namespace rolling_map
