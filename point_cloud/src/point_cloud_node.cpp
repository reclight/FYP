#include <ros/ros.h>
#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <boost/foreach.hpp>
#include <mask_rcnn_ros/Result.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include<image_transport/image_transport.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandlerXYZ;

class Segmentate_PointCloud
{
  public:
  Segmentate_PointCloud(ros::NodeHandle& nh)
	{
    image_pub = nh.advertise<sensor_msgs::Image>("point_cloud_node/Image", 1, true);
    point_cloud_all_pub = nh.advertise<PointCloud>("point_cloud_node/PointCloudAll", 1, true);
    point_cloud_pub = nh.advertise<PointCloud>("point_cloud_node/PointCloud", 1, true);
    point_cloud_debug_pub = nh.advertise<PointCloud>("point_cloud_node/PointCloudDebug", 1, true);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("point_cloud_node/Poses", 1, true);
    dimension_pub = nh.advertise<geometry_msgs::Vector3>("point_cloud_node/Dimension", 1, true);

    mask_sub = nh.subscribe<mask_rcnn_ros::Result>("/mask_rcnn/result", 1, &Segmentate_PointCloud::mask_callback, this);	
		point_cloud_sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &Segmentate_PointCloud::pc_callback, this);
    object_sub = nh.subscribe<std_msgs::Int32>("/main_program/Object", 1, &Segmentate_PointCloud::object_callback, this);

    //declare publisher variable
    //PointCloud::Ptr pc (new PointCloud);
	}

  void mask_callback(const mask_rcnn_ros::Result result)
  {
    cv_bridge::CvImagePtr cv_ptr;
    mask_result = result;
  }

  void object_callback(const std_msgs::Int32 object)
  {
    chosen = object.data;
  }

  //Point cloud feedback
  void pc_callback(const PointCloud::ConstPtr& msg)
  {
    ros::Duration(0.1).sleep();
    ROS_INFO("ROS Topic detected");
    int i=0;
    geometry_msgs::PoseArray pose_published;
    sensor_msgs::Image image;

    PointCloud::Ptr pc_all (new PointCloud);
    //Publish the point cloud
    pc_all->header.frame_id = "camera_link";
    pc_all->height = msg->height;
    pc_all->width = msg->width;
    pc_all->points = msg-> points;
    pcl_conversions::toPCL(ros::Time::now(), pc_all->header.stamp);

    point_cloud_all_pub.publish(pc_all);
    
    PointCloud::Ptr pc_viz (new PointCloud);

    ROS_INFO("HELLO");

    if(!mask_result.class_ids.empty())
    {
      BOOST_FOREACH(int id, mask_result.class_ids)
      {
        ROS_INFO("HIIIII");
        //for visualization purpose
        if(i==0)
        {
          image = mask_result.masks[i];
        }
        else
        {
          int size = msg->width * msg->height;
          for (int mask=0; mask < size; mask++)
          {
            if(image.data[mask] || mask_result.masks[i].data[mask])
            {
              image.data[mask] = 255;
            }
          }
        }
        
        


        //For image segmentation
        ROS_INFO("i = %d, id= %d", i, id);
        if(id == chosen)
        {
          PointCloud::Ptr pc (new PointCloud);
          //Publish the point cloud
          pc->header.frame_id = "camera_link";
          pc->height = msg->height;
          pc->width = msg->width;

          ROS_INFO("Chosen: %d", chosen);
          //Extracting the mask[1]
          

          //Handle the point cloud
          int mask_pointer=0;
          //ROS_INFO("Cloud: width = %d, height = %d, number = %d\n", msg->width, msg->height,i);
          BOOST_FOREACH (const pcl::PointXYZRGB& pcloud, msg->points)
          {
            pcl::PointXYZRGB pt = pcloud;

            if(mask_result.masks[i].data[mask_pointer]==0)
            {
              pt.x = std::numeric_limits<float>::quiet_NaN();
              pt.y = std::numeric_limits<float>::quiet_NaN();
              pt.z = std::numeric_limits<float>::quiet_NaN();
            }
            
            //ROS_INFO ("\txyz : (%f, %f, %f)\n", pt.x, pt.y, pt.z);

            /*uint32_t rgb = *reinterpret_cast<int*>(pt.rgb);
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            */
            //ROS_INFO("\trgb : (%d, %d, %d)\n", pt.r, pt.g, pt.b);
            pc->points.push_back (pt);
            
            mask_pointer++;
            
          }

          //Publish the point cloud
          pcl_conversions::toPCL(ros::Time::now(), pc->header.stamp);

          pose_published.poses.push_back(PointCloud_MBV(pc));
        }
        i++;
      }
    

      //for visualization purpose
      int mask_pointer=0;

      BOOST_FOREACH (const pcl::PointXYZRGB& pcloud, msg->points)
      {
        pcl::PointXYZRGB pt_viz = pcloud;

        if(image.data[mask_pointer]==0)
        {
          pt_viz.x = std::numeric_limits<float>::quiet_NaN();
          pt_viz.y = std::numeric_limits<float>::quiet_NaN();
          pt_viz.z = std::numeric_limits<float>::quiet_NaN();
        }

        pc_viz -> points.push_back(pt_viz);
        mask_pointer++;
      }

      image_pub.publish(image);

      pc_viz->header.frame_id = "camera_link";
      pc_viz->height = msg->height;
      pc_viz->width = msg->width;
      pcl_conversions::toPCL(ros::Time::now(), pc_viz->header.stamp);
      point_cloud_pub.publish(pc_viz);

      pose_published.header.frame_id = "camera_link";
      pose_published.header.stamp=ros::Time::now();
      pose_pub.publish(pose_published);
    }
  }

  //find the minimum bounding box
  geometry_msgs::Pose PointCloud_MBV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& Segmented)
  {
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented = Segmented;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = Segmented;

    pcl::io::savePCDFileASCII ("/home/reclight/test_pcd.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented (new pcl::PointCloud<pcl::PointXYZRGB> ());
    if (pcl::io::loadPCDFile ("/home/reclight/test_pcd.pcd", *cloudSegmented) == -1)
    //if (pcl::io::loadPCDFile ("/home/reclight/min_cut_segmentation_tutorial.pcd", *cloudSegmented) == -1)
    {
      
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudSegmented, *cloudSegmented, indices);


    //point cloud filtering
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud (cloudSegmented);
    ror.setRadiusSearch (0.008);
    ror.setMinNeighborsInRadius (150);
    ror.filter (*cloud_filtered);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (400);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    /*pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud (cloudSegmented);
    sor2.setLeafSize (0.01f, 0.01f, 0.01f);
    sor2.filter (*cloud_filtered);*/


    /*pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud_filtered);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);*/

    /*std::cout << cloudSegmented <<std::endl;
    std::cout << position_OBB <<std::endl;
    ROS_INFO("-----------------------------------------------------");
    
    */

    cloud_filtered->header.frame_id = "camera_link";
    pcl_conversions::toPCL(ros::Time::now(), cloud_filtered->header.stamp);
    point_cloud_debug_pub.publish(cloud_filtered);

    //Finding minimum volume bounding box (aka pose)
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_filtered, pcaCentroid);
    
    
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_filtered, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                ///    the signs are different and the box doesn't get correctly oriented in some cases.

    
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_filtered, *cloudPointsProjected, projectionTransform);
    
      
    //point_cloud_debug_pub.publish(*cloudPointsProjected);
    

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    geometry_msgs::Vector3 boxDimension;
    boxDimension.x = maxPoint.x - minPoint.x;
    boxDimension.y = maxPoint.y - minPoint.y;
    boxDimension.z = maxPoint.z - minPoint.z;

    bool rect = false;

    /*if (boxDimension.y / boxDimension.z > 0.9)
    {
      boxDimension.y = boxDimension.y / 1.414214;
      boxDimension.z = boxDimension.z / 1.414214;
      
      Eigen::Matrix3f rotate_45;
      rotate_45 << 1, 0, 0,
                  0, 0.7071, 0.7071,
                  0, -0.7071, 0.7071;

      Eigen::Matrix3f eigenVectorsPCA = eigenVectorsPCA * rotate_45;
      bboxQuaternion = eigenVectorsPCA;
    }*/

    dimension_pub.publish(boxDimension);

    //ROS_INFO("maxPoint.x : %f, maxPoint.y : %f, maxPoint.z: %f", maxPoint.x, maxPoint.y, maxPoint.z);
    //ROS_INFO("minPoint.x : %f, minPoint.y : %f, minPoint.z: %f", minPoint.x, minPoint.y, minPoint.z);
    //ROS_INFO("x: %f, y: %f, z: %f", maxPoint.x-minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z);

    geometry_msgs::Pose objectpose;
    geometry_msgs::Point position;

    objectpose.position.x = bboxTransform[0];
    objectpose.position.y = bboxTransform[1];
    objectpose.position.z = bboxTransform[2];

    //tf::quaternionEigenToMsg(bboxQuaternion, objectpose.pose.orientation);

    auto euler = bboxQuaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    //std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler[0] << std::endl;
    //std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler << std::endl;
    //std::cout << "Eigen Vector"<< std::endl << eigenVectorsPCA << std::endl;
    if(euler[2]>0)
    {
      Eigen::Matrix3f rotate_z;
      rotate_z << -1, 0, 0,
                  0, -1, 0,
                  0, 0, 1;

      Eigen::Matrix3f eigenVectorTransformed = eigenVectorsPCA * rotate_z;
      bboxQuaternion = eigenVectorTransformed;
      //std::cout << "Eigen Vector trasnformed"<< std::endl << eigenVectorTransformed << std::endl;


      auto euler = bboxQuaternion.toRotationMatrix().eulerAngles(0, 1, 2);
      //std::cout << "Euler from quaternion in roll, pitch, yaw trasnformed : "<< std::endl << euler << std::endl;
    }

    objectpose.orientation.x = bboxQuaternion.x();
    objectpose.orientation.y = bboxQuaternion.y();//bboxQuaternion[1];
    objectpose.orientation.z = bboxQuaternion.z();//bboxQuaternion[2];
    objectpose.orientation.w = bboxQuaternion.w();//bboxQuaternion[3];
    return objectpose;
  }

  private:
		ros::Subscriber point_cloud_sub;
    ros::Subscriber mask_sub;
    ros::Subscriber object_sub;

    ros::Publisher image_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher point_cloud_all_pub;
    ros::Publisher point_cloud_debug_pub;
    ros::Publisher pose_pub;
    ros::Publisher dimension_pub;

    std::string object_type;

    int chosen = 1;

    //PointCloud::Ptr pc;
    mask_rcnn_ros::Result mask_result;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  Segmentate_PointCloud pc_seg(nh);
  ROS_INFO("Segmentate_PointCloud initiated");
  ros::spin();
}