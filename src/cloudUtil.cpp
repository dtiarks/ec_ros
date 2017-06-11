/*
 * cloudUtil.cpp
 * 
 * Copyright 2016 daniel <daniel@daniel-Inspiron>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include "cloudUtil.h"
#include "ec_ros.h"

void align::passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                            const std::string& axis, float minPoint, float maxPoint)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(sourceCloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(minPoint, maxPoint);
  pass.filter(*filteredCloud);
}

void align::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                      float leafSize)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(sourceCloud);
  sor.setLeafSize(leafSize, leafSize, leafSize);
  sor.filter(*filteredCloud);
}

void align::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene,pcl::PointCloud<pcl::PointNormal>::Ptr& normals)
{
  // Estimate normals for scene
  ROS_INFO ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*normals);
}

void align::featureEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,pcl::PointCloud<pcl::PointNormal>::Ptr& input_normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features)
{
  //~ // Estimate features
  ROS_INFO ("Estimating features...\n");
  pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::PointNormal,pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (input);
  fest.setInputNormals (input_normals);
  fest.compute (*features);
}

void align::euclideanSegment(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, std::vector<pcl::PointIndices> &cluster_indices)
{
    //~ FPS_CALC_BEGIN;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (50000);
    //ec.setMaxClusterSize (400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    //~ FPS_CALC_END("euclideanSegmentation");
}

uint32_t align::genColors(int nr)
{
  int i;

  for(i=1;i<nr;i+=(int)255/nr)
  {
    rgbs[i][0]=(uint8_t)255/i;
    rgbs[i][1]=(uint8_t)255/((i+255/nr)%255);
    rgbs[i][2]=(uint8_t)255/((i+2*255/(nr))%255);
  }
}

uint32_t align::segmentColor (int segment)
{

  uint32_t rgb = ((uint32_t)rgbs[segment%SEGMENT_COLORS][0] << 16 | (uint32_t)rgbs[segment%SEGMENT_COLORS][1] << 8 | (uint32_t)rgbs[segment%SEGMENT_COLORS][2]);

  return rgb;
}

void align::extractSegmentCluster (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, 
                                   const std::vector<pcl::PointIndices> cluster_indices, 
                                   const int segment_index, 
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result)
{
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    
    uint32_t rgb = segmentColor(segment_index);
    
    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
    {
      pcl::PointXYZRGB point;
      point.x = cloud->points[segmented_indices.indices[i]].x;
      point.y = cloud->points[segmented_indices.indices[i]].y;
      point.z = cloud->points[segmented_indices.indices[i]].z;
      point.rgb = *reinterpret_cast<float*>(&rgb);
      result->points.push_back (point);
    }
    result->width = pcl::uint32_t (result->points.size ());
    result->height = 1;
    result->is_dense = false;
}
