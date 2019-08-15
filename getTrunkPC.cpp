#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <cmath>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <fstream>
//#indlude <>

using namespace std;

typedef pcl::PointXYZ PointT;

float shiftPC(pcl::PointCloud<PointT>::Ptr cloud){
  //get z_maz: tree height
  float tree_height;
  float x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = cloud->points[0].x;
  x_max = cloud->points[0].x;
  y_min = cloud->points[0].y;
  y_max = cloud->points[0].y;
  z_min = cloud->points[0].z;
  z_max = cloud->points[0].z;
  //construct color for cloud
  tree_height = cloud->points[1].z;
  for (int i = 0; i < cloud->points.size(); ++i){
    if (cloud->points[i].x < x_min){
        x_min = cloud->points[i].x;}
    if (cloud->points[i].x > x_max){
        x_max = cloud->points[i].x;}
    if (cloud->points[i].y < y_min){
        y_min = cloud->points[i].y;}
    if (cloud->points[i].y > y_max){
        y_max = cloud->points[i].y;}
    if (cloud->points[i].z < z_min){
        z_min = cloud->points[i].z;}
    if (cloud->points[i].z > z_max){
        z_max = cloud->points[i].z;}}

  tree_height = z_max - z_min;
  std::cout << "tree height: " << tree_height << std::endl;
  std::cout << "x range: " << x_min << " ~ " << x_max << std::endl; 
  std::cout << "y range: " << y_min << " ~ " << y_max << std::endl; 
  std::cout << "z range: " << z_min << " ~ " << z_max << std::endl;
  //shift point cloud to origin
  for (int i = 0; i < cloud->points.size(); ++i){
    cloud->points[i].x = cloud->points[i].x - x_min;
    cloud->points[i].y = cloud->points[i].y - y_min;
    cloud->points[i].z = cloud->points[i].z - z_min;}
  return tree_height;
}

int
main (int argc, char *argv[])
{
  float diameter_offset; //offset on stem model cylinder to cut out stem PC
  diameter_offset = atof(argv[2]);
  // All the objects needed
  pcl::PCDReader reader;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_1;
  pcl::NormalEstimation<PointT, pcl::Normal> ne_2;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_1;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_2; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree_1 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_2 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_3 (new pcl::search::KdTree<PointT> ());
  pcl::search::KdTree<PointT>::Ptr tree_4 (new pcl::search::KdTree<PointT> ());
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_1_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_3 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr rest_cloud_4 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr layer_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_1 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_temp_2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_3 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_4 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_1 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_1 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_2 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_3 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_3 (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder_4 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder_4 (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr layer_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointT>::Ptr inlier_cloud_1 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_2 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_3 (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr inlier_cloud_4 (new pcl::PointCloud<PointT> ());

  pcl::PointXYZ basic_point;
  pcl::PointXYZRGB point;
  // Read in the cloud data
  //reader.read ("../data/rotPlant.pcd", *cloud);
  reader.read (argv[1], *cloud);
  std::cerr << "Input PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  float treeheight;

#if 0
  //----------------------------------------------process 1---------------------
  //RANSAC
  ne_1.setSearchMethod (tree_1);
  ne_1.setInputCloud (cloud);
  ne_1.setKSearch (50);
  ne_1.compute (*cloud_normals_1);

  //Create the segmentation object for cylinder segmentation and set all the parameters
  seg_1.setOptimizeCoefficients (true);
  seg_1.setModelType (pcl::SACMODEL_CYLINDER);
  seg_1.setMethodType (pcl::SAC_RANSAC);
  seg_1.setNormalDistanceWeight (0.1);//0.1
  seg_1.setMaxIterations (10000); //10000
  seg_1.setDistanceThreshold (0.04);
  seg_1.setRadiusLimits (trunkradiuslimitlower, trunkradiuslimitupper); //(0, 0.1)//(0.03, )
  //seg_1.setRadiusLimits (0.05, 0.06);
  seg_1.setInputCloud (cloud);
  seg_1.setInputNormals (cloud_normals_1);

  seg_1.segment (*inliers_cylinder_1, *coefficients_cylinder_1);
  //----------------------------------------------process 1---------------------
  // find inlier in this layer
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder_1);
  extract.setNegative (false);
  extract.filter (*inlier_cloud_1);
 

  extract.setNegative (true);
  extract.filter (*rest_cloud_1);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals_1);
  extract_normals.setIndices (inliers_cylinder_1);
  extract_normals.filter (*cloud_normals_2);

  writer.write ("rest_cloud.pcd", *rest_cloud_1, false);

  std::cout << "outliers numbers: " << rest_cloud_1->points.size() << std::endl;
  std::cerr << "Cylinder 1 coefficients: " << *coefficients_cylinder_1 << std::endl;
  

#endif
  //color cloud plot
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_cloud_color_handler (cloud, 255, 0, 0); // Red
  viewer->addPointCloud (cloud, org_cloud_color_handler, "org_cloud");
  treeheight = shiftPC(cloud);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> shift_cloud_color_handler (cloud, 255, 255, 0); // Red
  viewer->addPointCloud (cloud, shift_cloud_color_handler, "shift_cloud");


  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //return (viewer);


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
