/*********************************************************************
*
*  © (or copyright) 2022. Triad National Security, LLC.
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
* Author: Alex Navarro
*********************************************************************/

#include <cuda_runtime.h>
#include "rolling_map.h"
#include "cuda_safe.cuh"
#include "cuda_voxel_grid.cuh"
#include <thrust/remove.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

// ================================================================================================
// ================================================================================================

namespace rolling_map{

// ================================================================================================
// ================================================================================================

__global__ void getIndexCuda(rolling_map::cudaVoxelGrid* voxel_grid, const rolling_map::Coord idx, bool* value){
    *value = voxel_grid->getVoxel(idx);
}

bool RollingMap::atIndex(int x, int y, int z, bool& value){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

    if (not checkIndex(x, y, z)) return false;

    bool* val;
    CUDA_SAFE(cudaMalloc(&val, sizeof(bool)));
    getIndexCuda<<<1,1>>>(this->d_voxel_grid_, {x,y,z}, val);
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError(); /* getIndexCuda */);

    CUDA_SAFE(cudaMemcpy(&value, val, sizeof(bool), cudaMemcpyDeviceToHost));
    CUDA_SAFE(cudaFree(val));

    bool ret_val = cuda_ok;
    cuda_ok = true;
    return ret_val;
}

bool RollingMap::atPosition(float xp, float yp, float zp, bool& value){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_);

    int x, y, z;
    index(xp, yp, zp, x, y, z);

    return atIndex(x, y, z, value);
}

// ================================================================================================
// ================================================================================================

void RollingMap::clearAll(){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_);
    CUDA_SAFE(cudaMemset(d_voxel_grid_, 0x00, voxel_grid_size_bytes_));
}

// ================================================================================================
// ================================================================================================

void RollingMap::insertCloud(const std::vector<pcl::PointXYZ> &scancloud, const pcl::PointXYZ &sensorOrigin){
    std::lock_guard<std::shared_timed_mutex> write_lock(map_mutex_); 

    Coord sensor_voxel_idx;
    pcl::PointXYZ start_voxel_loc;

    // Get index of sensor posistion
    if(!toIndex(sensorOrigin.x, sensorOrigin.y, sensorOrigin.z, sensor_voxel_idx.x, sensor_voxel_idx.y, sensor_voxel_idx.z))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because it is not contained in the map bounds. Map bounds: (" 
        << getMinXP() << ", " << getMinYP() << ", " << getMinZP() << ") to (" << getMaxXP() << ", " << getMaxYP() << ", " << getMaxZP() 
        << "). sensor position: (" << sensorOrigin.x << ", " << sensorOrigin.y << ", " << sensorOrigin.z << ").");
    return; 
    }

    // Get position of start voxel
    if(!toPosition(sensor_voxel_idx.x, sensor_voxel_idx.y, sensor_voxel_idx.z, start_voxel_loc.x, start_voxel_loc.y, start_voxel_loc.z))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Cannot cast ray from sensor because sensor voxel is not contained in the map bounds. Map bounds: (0, 0, 0) to " 
        << getMaxXI() << ", " << getMaxYI() << ", " << getMaxZI() 
        << "). sensor position: (" << sensor_voxel_idx.x << ", " << sensor_voxel_idx.y << ", " << sensor_voxel_idx.z << ").");
    return; 
    }

    // Cast all rays on gpu for multithreading
    if(!castRays(scancloud, sensorOrigin, start_voxel_loc))
        ROS_ERROR_STREAM_THROTTLE(1.0, "RollingMap: Error in cuda castRays function.");
}

// ================================================================================================
// ================================================================================================

__global__ void reduceGrid(cudaVoxelGrid* grid, Coord* pointcloud){
    const size_t voxel_x_idx = threadIdx.x + blockIdx.x * blockDim.x;
    const size_t voxel_y_idx = threadIdx.y + blockIdx.y * blockDim.y;
    const size_t voxel_z_idx = threadIdx.z + blockIdx.z * blockDim.z;

    if (voxel_x_idx >= grid->width || voxel_y_idx >= grid->width || voxel_z_idx >= grid->height) return;

    size_t idx = grid->width * grid->width * voxel_z_idx + grid->width * voxel_y_idx + voxel_x_idx;

    // Clamp the voxel value to legal limits
    voxel_block_t& voxel = grid->voxels[idx];
    if (voxel > grid->probability_maximum)  
        voxel = grid->probability_maximum;
    else if (voxel < 0)     
        voxel = 0;
    
    // Mark as occupied if over the threshold
    if (voxel > grid->probability_threshold){
        Coord& c = pointcloud[idx];
        c.x = voxel_x_idx;
        c.y = voxel_y_idx;
        c.z = voxel_z_idx;
    }
}

// ================================================================================================
// ================================================================================================

// Define struct for check if a Coord is zero on the device
struct is_zero{
    __device__ __host__
    bool operator()(const Coord& c) const noexcept{
        return c.x == 0 && c.y == 0 && c.z == 0;
    }
};

std::vector<Coord> RollingMap::getMap(){
    std::shared_lock<std::shared_timed_mutex> read_lock(map_mutex_); 

    // Create a pointcloud where all points are (0, 0, 0)
    thrust::device_vector<Coord> device_pointcloud(width_*width_*height_);

    // Fill any point that is occupied with its integer coordinate
    dim3 block_dim(8, 8, 4);
    dim3 grid_dim(width_/block_dim.x + 1, width_/block_dim.y + 1, height_/block_dim.z + 1);
    reduceGrid<<<grid_dim, block_dim>>>(d_voxel_grid_, device_pointcloud.data().get());
    cudaDeviceSynchronize();
    CUDA_SAFE(cudaGetLastError());

    // Remove any point that is still zero after the process
    auto new_end = thrust::remove_if(device_pointcloud.begin(), device_pointcloud.end(), is_zero());
    device_pointcloud.erase(new_end, device_pointcloud.end());
    std::vector<Coord> mapcloud(device_pointcloud.size());
    thrust::copy(device_pointcloud.begin(), device_pointcloud.end(), mapcloud.begin());

    return mapcloud;
}


} // end namespace rolling_map
