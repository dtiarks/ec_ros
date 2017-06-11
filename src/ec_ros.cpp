/*
 * align.cpp
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


align::align()
{
	 //~ refCloud = new pcl::PCLPointCloud2; 
	 refCloud = new pcl::PointCloud<pcl::PointXYZ>;
	 leaf=0.005f;

  genColors(SEGMENT_COLORS);

	 mode=ICP_REGISTER;
   ROS_INFO("Using mode: %d", mode);
}

align::align(std::string refFile)
{
	 refCloud = new pcl::PointCloud<pcl::PointXYZ>;
	 leaf=0.05f;
	 loadRef(refFile);

   genColors(SEGMENT_COLORS);

	 mode=ICP_REGISTER;
   ROS_INFO("Using mode: %d", mode);
}

align::align(std::string refFile,const int m)
{
	 refCloud = new pcl::PointCloud<pcl::PointXYZ>;
	 leaf=0.005f;
	 loadRef(refFile);

   genColors(SEGMENT_COLORS);
	 
	 mode=m;
   ROS_INFO("Using mode: %d", mode);
}

align::~align()
{
	
}

void align::loadRef(std::string refFile)
{
  ROS_INFO("Loading ref cloud: %s", refFile.c_str());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (refFile, *refCloud) == -1)
  {
	  ROS_ERROR("Ref cloud not found: %s", refFile.c_str());
	  
  }
  ROS_INFO("Size: %d", refCloud->size());
}

void align::publishScene(ros::Publisher *pub)
{
  output_msg.header.frame_id="kinect2_ir_optical_frame";
  pub->publish(output_msg);
}

void align::publishSegmented(ros::Publisher *pub)
{
  segmented_msg.header.frame_id="kinect2_ir_optical_frame";
  pub->publish(segmented_msg);
}

void align::publishAligned(ros::Publisher *pub)
{
  aligned_msg.header.frame_id="kinect2_ir_optical_frame";
  pub->publish(aligned_msg);
}

void align::publishRef(ros::Publisher *pub)
{
  pcl::PCLPointCloud2 out;
  sensor_msgs::PointCloud2 msg;
  
  pcl::toPCLPointCloud2(*refCloud, out);
  pcl_conversions::fromPCL(out, msg);
  msg.header.frame_id="kinect2_ir_optical_frame";
  pub->publish(msg);
}

void align::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sag (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>) ;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented (new pcl::PointCloud<pcl::PointXYZRGB>) ;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object ;
  object=refCloud->makeShared();

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;

  pcl::PointCloud<pcl::PointXYZRGB> concCloud;
  
  pcl::PointCloud<pcl::PointNormal>::Ptr scene_normals (new pcl::PointCloud<pcl::PointNormal>) ;
  pcl::PointCloud<pcl::PointNormal>::Ptr object_normals (new pcl::PointCloud<pcl::PointNormal>) ;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);


  //Conversion
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::fromPCLPointCloud2(*cloud, *cloud_sag);
  
  //Filtering
  //~ passthroughFilter(cloud_sag,filteredCloud,"z",0.6,1);
  //~ passthroughFilter(cloud_sag,filteredCloud,"z",0.2,0.5);
  //~ passthroughFilter(filteredCloud,filteredCloud,"y",-0.05,0.2);
  voxelFilter(cloud_sag,filteredCloud,0.01f);
  
  
  switch(mode)
  {
  	case ICP_REGISTER:
      normalEstimation(filteredCloud,scene_normals);
      normalEstimation(object,object_normals);
    
      featureEstimation(object,object_normals,object_features);
      featureEstimation(filteredCloud,scene_normals,scene_features);

  		runICP(object,filteredCloud,object_features,scene_features,object_aligned);
  		break;
  	case EUCLIDEAN_SEGMENT:
      
  		euclideanSegment (filteredCloud, cluster_indices);
  		int c_size = cluster_indices.size();
      int i;
      
      ROS_INFO("Detected %d cloud segments", c_size);

      for(i=0;i<c_size;i++)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        extractSegmentCluster(filteredCloud,cluster_indices,i,segmentCloud);
        concCloud+=*segmentCloud;
        //clouds.push_back(*segmentCloud);

      }
    
		
		break;
		
  }
  
  // Convert to ROS data type
  
  pcl::PCLPointCloud2 out;
  pcl::PCLPointCloud2 out_aligned;
  pcl::PCLPointCloud2 out_segmented;
  pcl::toPCLPointCloud2(*filteredCloud, out);
  pcl::toPCLPointCloud2(*object_aligned, out_aligned);
  pcl::toPCLPointCloud2(concCloud, out_segmented);
  pcl_conversions::fromPCL(out, output_msg);
  pcl_conversions::fromPCL(out_aligned, aligned_msg);
  pcl_conversions::fromPCL(out_segmented, segmented_msg);

}

