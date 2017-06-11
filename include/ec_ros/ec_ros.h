/*
 * align.h
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



#ifndef ALIGN_H
#define ALIGN_H

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <iostream>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#define ICP_REGISTER 0
#define EUCLIDEAN_SEGMENT 1
#define SEGMENT_COLORS 100

//Futere: make template that defines the point type from the beginning
class align
{
	public:
		align();
		align(std::string refFile);
		align(std::string refFile,const int m);
		virtual ~align();
		
		
		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
		void publishRef(ros::Publisher *pub);
		void publishScene(ros::Publisher *pub);
		void publishAligned(ros::Publisher *pub);
		void publishSegmented(ros::Publisher *pub);
		void loadRef(std::string refFile);
		
		void passthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                            const std::string& axis, float minPoint, float maxPoint);
        void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourceCloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud,
                      float leafSize);
        void normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& scene,pcl::PointCloud<pcl::PointNormal>::Ptr& normals);
        void featureEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,pcl::PointCloud<pcl::PointNormal>::Ptr& input_normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features);
        
        void runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& object,pcl::PointCloud<pcl::PointXYZ>::Ptr& scene,pcl::PointCloud<pcl::FPFHSignature33>::Ptr& object_features,
                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr& scene_features,pcl::PointCloud<pcl::PointXYZ>::Ptr& object_aligned);
		void euclideanSegment(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         std::vector<pcl::PointIndices> &cluster_indices);
        void extractSegmentCluster (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const std::vector<pcl::PointIndices> cluster_indices, const int segment_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& result);
        uint32_t segmentColor (int segment);
        uint32_t genColors(int nr);

        ros::NodeHandle nh;
        
	private:
		pcl::PointCloud<pcl::PointXYZ> *refCloud;
		pcl::PointCloud<pcl::PointXYZ> *refAligned;
		pcl::PointCloud<pcl::PointNormal> *refNormals;
		pcl::PointCloud<pcl::PointXYZ> *sceneCloud;
		pcl::PointCloud<pcl::PointNormal> *sceneNormals;
		
		sensor_msgs::PointCloud2 output_msg;
		sensor_msgs::PointCloud2 aligned_msg;
		sensor_msgs::PointCloud2 segmented_msg;
		
		uint8_t rgbs[SEGMENT_COLORS][3];

		int clusters_detected;
		
		float leaf;
		int mode;
		
};

#endif /* ALIGN_H */ 
