/*
 * registerICP.cpp
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


#include "ec_ros.h"


void align::runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& object,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& scene,
                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr& object_features,
                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr& scene_features,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& object_aligned)
{
  //~ pcl::PointCloud<pcl::PointXYZ>::Ptr object_aigned (new pcl::PointCloud<pcl::PointXYZ>);
  // Perform alignment
  ROS_INFO ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (5000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.12f); // Required inlier fraction for accepting a pose hypothesis
  
  align.align (*object_aligned);
  
  
  if (align.hasConverged ())
  {
	  ROS_INFO("Alignment succesful!");
  }else
  {
	  ROS_ERROR("Alignment failed!");
  }
}
