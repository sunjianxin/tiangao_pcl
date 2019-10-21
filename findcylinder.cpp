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
//
#define OFFSET 0.02

using namespace std;

typedef pcl::PointXYZ PointT;

float getPoint2LineDistance(float x, float y, float z, float line_org_x, float line_org_y, float line_org_z, float line_dir_x, float line_dir_y, float line_dir_z) {
  // get vector from line org to point as diff vector
  float x_diff = x - line_org_x;
  float y_diff = y - line_org_y;
  float z_diff = z - line_org_z;
  // get outer product of diff vector and line, and get its norm
  float norm = sqrt(pow(x_diff*line_dir_y-y_diff*line_dir_x, 2) + 
               pow(z_diff*line_dir_x-x_diff*line_dir_z, 2) +
               pow(y_diff*line_dir_z-z_diff*line_dir_y, 2));
  
  // get point to line distance
  return norm/sqrt(pow(line_dir_x, 2) + pow(line_dir_y, 2) + pow(line_dir_z, 2));
}

void getStemPC(pcl::PointCloud<PointT>::Ptr org_cloud, pcl::PointCloud<PointT>::Ptr stem_cloud, pcl::PointCloud<PointT>::Ptr rest_cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder, float offset) {
  int count_stem = 0;
  int count_rest = 0;
  for (int i = 0; i < org_cloud->points.size(); ++i){
    if (getPoint2LineDistance(org_cloud->points[i].x,
                              org_cloud->points[i].y,
                              org_cloud->points[i].z,
                              coefficients_cylinder->values[0],
                              coefficients_cylinder->values[1],
                              coefficients_cylinder->values[2],
                              coefficients_cylinder->values[3],
                              coefficients_cylinder->values[4],
                              coefficients_cylinder->values[5]
                              ) > (coefficients_cylinder->values[6] + offset)){
      stem_cloud->points.push_back(org_cloud->points[i]);
      count_stem ++;
    } else {
      rest_cloud->points.push_back(org_cloud->points[i]);
      count_rest ++;
    }
  }
  std::cout << "count_stem: " << count_stem << std::endl;
  std::cout << "count_rest: " << count_rest << std::endl;
  stem_cloud->width = count_stem;
  rest_cloud->width = count_rest;
  stem_cloud->height = 1;
  rest_cloud->height = 1;
  stem_cloud->points.resize(count_stem);
  rest_cloud->points.resize(count_rest);
}

void getStemPC_RGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest_cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder, float offset) {
  int count_stem = 0;
  int count_rest = 0;
  for (int i = 0; i < org_cloud->points.size(); ++i){
    if (getPoint2LineDistance(org_cloud->points[i].x,
                              org_cloud->points[i].y,
                              org_cloud->points[i].z,
                              coefficients_cylinder->values[0],
                              coefficients_cylinder->values[1],
                              coefficients_cylinder->values[2],
                              coefficients_cylinder->values[3],
                              coefficients_cylinder->values[4],
                              coefficients_cylinder->values[5]
                              ) > (coefficients_cylinder->values[6] + offset)){
      stem_cloud->points.push_back(org_cloud->points[i]);
      count_stem ++;
    } else {
      rest_cloud->points.push_back(org_cloud->points[i]);
      count_rest ++;
    }
  }
  std::cout << "count_stem: " << count_stem << std::endl;
  std::cout << "count_rest: " << count_rest << std::endl;
  stem_cloud->width = count_stem;
  rest_cloud->width = count_rest;
  stem_cloud->height = 1;
  rest_cloud->height = 1;
  stem_cloud->points.resize(count_stem);
  rest_cloud->points.resize(count_rest);
}

int
main (int argc, char *argv[])
{
  float res_lower; //resolution
  res_lower = atof(argv[1]);
  float res_upper; //resolution
  res_upper = atof(argv[2]);
  //float cutOutBundary;
  //cutOutBundary = atof(argv[3]);
  // All the objects needed
  float trunkradiuslimitlower;
  trunkradiuslimitlower = atof(argv[3]);
  float trunkradiuslimitupper;
  trunkradiuslimitupper = atof(argv[4]);
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
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
  //reader.read ("../data/rotPlant.pcd", *cloud_rgb);
  reader.read ("../data/01_D1N3_2019-05-27.pcd", *cloud);
  reader.read ("../data/01_D1N3_2019-05-27.pcd", *cloud_rgb);
  std::cerr << "Input PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  std::cout << "data1-x: " << cloud->points[1].x << std::endl;
  std::cout << "data1-y: " << cloud->points[1].y << std::endl;
  std::cout << "data1-z: " << cloud->points[1].z << std::endl;
 
  int j = 0;
  int count = 0;
  int inter_threshhold = 100; //point number threshhold for making next iteration decision
  bool second = false; //indicator of whether second iteration exist
  bool third = false; //indicator of whether third iteration exist
  float cylength = 0.0;
  float temp_length;
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
  uint8_t r(255), g(15), b(15);
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
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
        z_max = cloud->points[i].z;}

    if (cloud->points[i].z > tree_height){
      tree_height = cloud->points[i].z;}}
  std::cout << "tree height: " << tree_height << std::endl;
  std::cout << "x range: " << x_min << " ~ " << x_max << std::endl; 
  std::cout << "y range: " << y_min << " ~ " << y_max << std::endl; 
  std::cout << "z range: " << z_min << " ~ " << z_max << std::endl;
  //shift point cloud to origin
  for (int i = 0; i < cloud->points.size(); ++i){
    cloud->points[i].x = cloud->points[i].x - x_min;
    cloud->points[i].y = cloud->points[i].y - y_min;
    cloud->points[i].z = cloud->points[i].z - z_min;
    cloud_rgb->points[i].x = cloud_rgb->points[i].x - x_min;
    cloud_rgb->points[i].y = cloud_rgb->points[i].y - y_min;
    cloud_rgb->points[i].z = cloud_rgb->points[i].z - z_min;}
 
  //float d = 0.15; //cylinder lay thinkness
  int layer_number = int(ceil(tree_height/(res_upper-res_lower)));

#if 1 
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
  for (int i = 0; i < cloud->points.size(); ++i){
      count += 1;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      //std::cout << cloud_rgb->points[i].rgb << std::endl;
      //std::cout << *reinterpret_cast<uint32_t*>(&cloud_rgb->points[i].rgb) << std::endl;
      point.rgb = *reinterpret_cast<float*>(&rgb);
      layer_cloud_rgb->points.push_back(point);}
  layer_cloud_rgb->width = count;
  layer_cloud_rgb->height = 1;
  layer_cloud_rgb->points.resize(count);


  //color cloud plot
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, "sample cloud");
  // viewer->addPointCloud<pcl::PointXYZ> (inlier_cloud_1, "sample cloud 1");
  // viewer->addPointCloud<pcl::PointXYZ> (rest_cloud_1, "sample cloud 2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 1");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud 2");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //return (viewer);


  /*//cut current section outof the tree within a cutOutBundary

  float xp;
  float yp;

  count = 0;
  //float level[9] = {0, 0.25, 0.52, 0.85, 1.2, 1.45, 1.9, 2.0, 2.6};
  //for (int i = 0; i <9; ++i){
  pcl::PointCloud<PointT>::Ptr cloud_org (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cutted_cloud (new pcl::PointCloud<PointT>);
  reader.read ("tree_org.pcd", *cloud_org);
  for (int i = 0; i < cloud_org->points.size(); ++i){
    if (cloud_org->points[i].z > res_lower && cloud_org->points[i].z < res_upper){
      t = (cloud_org->points[i].z - z0)/dz;
      xp = cloud_org->points[i].x - dx*t;
      yp = cloud_org->points[i].y - dy*t;
      if (pow(xp-x0, 2)+pow(yp-y0, 2) < pow (radus, 2)+ cutOutBundary){
    count += 1;
    cutted_cloud->points.push_back(cloud_org->points[i]);}}}
  cutted_cloud->width = count;
  cutted_cloud->height = 1;
  cutted_cloud->points.resize(count);

  std::stringstream ss;
  ss << "cutted_tree_" << res_lower << "-" <<res_upper << ".pcd";
  writer.write<pcl::PointXYZ> (ss.str (), *cutted_cloud, false);
  std::cerr<< "in saving cutted tree" << std::endl;
  */
 
 /* 
  std::stringstream sss;
  sss << "cutted_tree_" << res_lower << "-" <<res_upper << ".txt";
  std::cout << sss.str() << std::endl;
  std::ofstream myfile;
  myfile.open(sss.str().c_str());
  for (int i = 0; i < 7; ++i){
    myfile << coefficients_cylinder_1->values[i] << "\n";}
  myfile << cylength << "\n";
  myfile.close();
  */
  
  pcl::ModelCoefficients cylinder_coeff;
  cylinder_coeff.values.resize (7);    // We need 7 values
  cylinder_coeff.values[0] = 0;
  cylinder_coeff.values[1] = 0;
  cylinder_coeff.values[2] = 0;
  cylinder_coeff.values[3] = -0.404;
  cylinder_coeff.values[4] = 0.0616425;
  cylinder_coeff.values[5] = 0.912679;
  cylinder_coeff.values[6] = 0.1;
  std::cout << "before" << endl;
  viewer->addCylinder(cylinder_coeff);
  std::cout << "after" << endl;


  viewer->addCoordinateSystem (1.0);
  
  //  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  //viewer.showCloud (cloud_temp);
  viewer->addCylinder(*coefficients_cylinder_1, "cylinder_1");

  //do transformation
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  // z axis: (0, 0, 1)
  // trunk cylinder direction:
  float x = cylinder_coeff.values[3];
  float y = cylinder_coeff.values[4];
  float z = cylinder_coeff.values[5];
  // calculate the cross product of z axis and trunk cylinder direction
  float x_norm = y;  //1*y - 0*z
  float y_norm = -x; //0*z - x
  float z_norm = 0;  //0*x - 0*y
  // calculate the unit vector of cross product vector
  float l = sqrt(pow(x_norm, 2)+pow(y_norm, 2));
  x_norm = x_norm/l;
  y_norm = y_norm/l;
  z_norm = z_norm/l;
    
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
  std::cout << "rotation: " << rotation << std::endl; 
  rotation(0, 1) = -z_norm;
  rotation(0, 2) = y_norm;
  rotation(1, 0) = z_norm;
  rotation(1, 2) = -x_norm;
  rotation(2, 0) = -y_norm;
  rotation(2, 1) = x_norm;

  std::cout << "norm: " << x << " " << y << " " << z << std::endl;
   
  float theta = acos(z);
  std::cout << "theta: " << theta << std::endl;  
  //theta = M_PI/4; // The angle of rotation in radians
  Eigen::Matrix3f result;
  std::cout << "result" << std::endl;
  std::cout << result << std::endl;
  std::cout << "rotation:" << std::endl;
  std::cout << rotation << std::endl;
  result = identity + std::sin(theta)*rotation + 2*pow(std::sin(theta/2), 2)*rotation*rotation;
  std::cout << "result" << std::endl;
  std::cout << result << std::endl;

  for (int i = 0; i < 3; i++){
     for (int j = 0; j < 3; j++){
        transform_1(i, j) = result(i, j);}} 

  pcl::PointCloud<PointT>::Ptr cloud_new (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud (*cloud, *cloud_new, transform_1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_new (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*cloud_rgb, *cloud_rgb_new, transform_1);

  //transform the starting point of the cylinder model
  pcl::PointCloud<PointT>::Ptr cyl_start_point (new pcl::PointCloud<PointT>);
  //std::vector<PointT, Eigen::aligned_allocator<PointT>> start_point;
  PointT start_point;
  start_point.x = coefficients_cylinder_1->values[0];
  start_point.y = coefficients_cylinder_1->values[1];
  start_point.z = coefficients_cylinder_1->values[2];

  cyl_start_point->points.push_back(start_point);
  cyl_start_point->width = 1;
  cyl_start_point->height = 1;
  cyl_start_point->points.resize(1);
  std::cout << "cyl point transform: " << cyl_start_point->points[0] << std::endl;
  pcl::PointCloud<PointT>::Ptr new_cyl_start_point (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud (*cyl_start_point, *new_cyl_start_point, transform_1);
  std::cout << "cyl point transform: " << new_cyl_start_point->points[0] << std::endl;


  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_new, 0, 255, 0); // Red
  //viewer->addPointCloud (cloud_new, transformed_cloud_color_handler, "transformed_cloud");

  //elevate the point cloud
  z_min = cloud_new->points[0].z;
  z_max = cloud_new->points[0].z;
  for (int i = 0; i < cloud_new->points.size(); ++i){
    if (cloud_new->points[i].z < z_min){
        z_min = cloud_new->points[i].z;}
    if (cloud_new->points[i].z > z_max){
        z_max = cloud_new->points[i].z;}}
  std::cout << "z range: " << z_min << " to " << z_max << std::endl;
  for (int i = 0; i < cloud_new->points.size(); ++i){
    cloud_new->points[i].z -= z_min;
    cloud_rgb_new->points[i].z -= z_min;}

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler_org (cloud_new, 255, 140, 0); // Orange
  viewer->addPointCloud (cloud_new, transformed_cloud_color_handler_org, "transformed_cloud_up");

  //Saved the final pcd file
  writer.write("rotPlant_reorientated.pcd", *cloud_new, false);

  //get PC of the stem
  std::cout << "1" << std::endl;
  pcl::ModelCoefficients::Ptr cylinder_coeff_tran (new pcl::ModelCoefficients);
  std::cout << "2" << std::endl;
  cylinder_coeff_tran->values.resize (7);    // We need 7 values
  std::cout << "3" << std::endl;
  cylinder_coeff_tran->values[0] = new_cyl_start_point->points[0].x;
  cylinder_coeff_tran->values[1] = new_cyl_start_point->points[0].y;
  cylinder_coeff_tran->values[2] = new_cyl_start_point->points[0].z;
  cylinder_coeff_tran->values[3] = 0;
  cylinder_coeff_tran->values[4] = 0;
  cylinder_coeff_tran->values[5] = 1;
  cylinder_coeff_tran->values[6] = 0.01;
  
  std::cout << "before" << endl;
  viewer->addCylinder(*cylinder_coeff_tran, "test");
  std::cout << "after" << endl;

  //pcl::PointCloud<PointT>::Ptr stem_cloud (new pcl::PointCloud<PointT>);
  //pcl::PointCloud<PointT>::Ptr rest_cloud (new pcl::PointCloud<PointT>);
  //getStemPC(cloud_new, stem_cloud, rest_cloud, cylinder_coeff_tran, OFFSET);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  getStemPC_RGB(cloud_rgb_new, stem_cloud, rest_cloud, cylinder_coeff_tran, OFFSET);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_2 (new pcl::visualization::PCLVisualizer ("3D Viewer 2"));
  viewer_2->setBackgroundColor (0, 0, 0);
  viewer_2->addCoordinateSystem (1.0);
  viewer_2->addCylinder(*cylinder_coeff_tran, "test");
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> stem_cloud_handler (stem_cloud, 255, 0, 0); // Red
  //viewer_2->addPointCloud (stem_cloud, stem_cloud_handler, "stem_cloud");
  viewer_2->addPointCloud (stem_cloud, "stem_cloud");
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rest_cloud_handler (stem_cloud, 0, 255, 0); // Green
  //viewer_2->addPointCloud (rest_cloud, rest_cloud_handler, "rest_cloud");
  viewer_2->addPointCloud (rest_cloud, "rest_cloud");
  std::cout << "size1: " << stem_cloud->size() << std::endl;
  std::cout << "size2: " << rest_cloud->size() << std::endl;

  //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (layer_cloud, cloud_normals_1, 10, 0.05, "normals");

  writer.write("leaf_cloud.pcd", *stem_cloud, false);
  writer.write("stem_cloud.pcd", *rest_cloud, false);





  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
