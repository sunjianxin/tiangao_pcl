#include <boost/thread/thread.hpp>
#include <cmath>
#include <fstream>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>

#include "functions.hpp"

#define OFFSET 0.04

using namespace std;

int main(int argc, char *argv[])
{
    float trunkradiuslimitlower;
    float trunkradiuslimitupper;
    string intputFn, outputPath;
    string leavesPcFn, stemPcFn;

    intputFn = argv[1];
    outputPath = argv[2];
    trunkradiuslimitlower = atof(argv[3]);
    trunkradiuslimitupper = atof(argv[4]);
    leavesPcFn = outputPath + "/allLeavesPc.ply";
    stemPcFn = outputPath + "/stemPc.ply";

    // All the objects needed
    // pcl::PCDReader reader;
    // pcl::PCDWriter writer;
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    pcl::NormalEstimation<PointT, pcl::Normal> ne_1;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_1;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree_1(new pcl::search::KdTree<PointT>());
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointT>::Ptr rest_cloud_1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr layer_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_1(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder_1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder_1(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr layer_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointT>::Ptr inlier_cloud_1(new pcl::PointCloud<PointT>());

    pcl::PointXYZ basic_point;
    pcl::PointXYZRGB point;
    // Read in the cloud data
    // reader.read("../../data/rotPlant_new.pcd", *cloud);
    // reader.read("../../data/rotPlant_new.pcd", *cloud_rgb);
    reader.read(intputFn, *cloud);
    reader.read(intputFn, *cloud_rgb);
    //reader.read ("../data/01_D1N3_2019-05-27.pcd", *cloud);
    //reader.read ("../data/01_D1N3_2019-05-27.pcd", *cloud_rgb);
    std::cerr << "Input PointCloud has: " << cloud->points.size() << " data points." << std::endl;
    //std::cout << "data1-x: " << cloud->points[1].x << std::endl;
    //std::cout << "data1-y: " << cloud->points[1].y << std::endl;
    //std::cout << "data1-z: " << cloud->points[1].z << std::endl;

    // Shift point cloud to the center of coordinate system
    shiftPC2Center(cloud, cloud_rgb);
    // Extract a new cylinder-shaped point cloud around the stem
    pcl::PointCloud<PointT>::Ptr rough_stem_cloud = getRoughStemPC(cloud, 0.25);
    pcl::PointCloud<PointT>::Ptr rough_stem_cloud_2 = getRoughStemPC(cloud, 1);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_0(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_0->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(rough_stem_cloud_2, 0, 255, 0); // Green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(rough_stem_cloud, 255, 0, 0);     // Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud, 0, 0, 255);               // Blue
    viewer_0->addPointCloud(cloud, blue, "org");
    viewer_0->addPointCloud(rough_stem_cloud_2, green, "rsc_2");
    viewer_0->addPointCloud(rough_stem_cloud, red, "rsc");
    viewer_0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rsc_2");
    viewer_0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rsc");
    viewer_0->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "org");
    viewer_0->addCoordinateSystem(1.0);
    viewer_0->initCameraParameters();

#if 1
    //----------------------------------------------process 1---------------------
    //RANSAC
    ne_1.setSearchMethod(tree_1);
    ne_1.setInputCloud(rough_stem_cloud);
    ne_1.setKSearch(50);
    ne_1.compute(*cloud_normals_1);

    //Create the segmentation object for cylinder segmentation and set all the parameters
    seg_1.setOptimizeCoefficients(true);
    seg_1.setModelType(pcl::SACMODEL_CYLINDER);
    seg_1.setMethodType(pcl::SAC_RANSAC);
    seg_1.setNormalDistanceWeight(0.1);
    seg_1.setMaxIterations(10000);
    seg_1.setDistanceThreshold(0.04);
    seg_1.setRadiusLimits(trunkradiuslimitlower, trunkradiuslimitupper);
    seg_1.setInputCloud(rough_stem_cloud);
    seg_1.setInputNormals(cloud_normals_1);

    seg_1.segment(*inliers_cylinder_1, *coefficients_cylinder_1);
    //----------------------------------------------process 1---------------------
    // find inlier in this layer
    extract.setInputCloud(rough_stem_cloud);
    extract.setIndices(inliers_cylinder_1);
    extract.setNegative(false);
    extract.filter(*inlier_cloud_1);

    extract.setNegative(true);
    extract.filter(*rest_cloud_1);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals_1);
    extract_normals.setIndices(inliers_cylinder_1);
    extract_normals.filter(*cloud_normals_2);

    // writer.write("rest_cloud.pcd", *rest_cloud_1, false);

    std::cout << "outliers numbers: " << rest_cloud_1->points.size() << std::endl;
    std::cerr << "Cylinder 1 coefficients: " << *coefficients_cylinder_1 << std::endl;

    viewer_0->addCylinder(*coefficients_cylinder_1, "fitted");

#endif
    //color cloud plot
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZRGB>(cloud_rgb, "sample cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();
    //return (viewer);

    // show fitted cylinder
    // viewer->addCylinder(*coefficients_cylinder_1, "cylinder_1");

    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize(7); // We need 7 values
    cylinder_coeff.values[0] = 0;
    cylinder_coeff.values[1] = 0;
    cylinder_coeff.values[2] = 0;
    cylinder_coeff.values[3] = -0.404;
    cylinder_coeff.values[4] = 0.0616425;
    cylinder_coeff.values[5] = 0.912679;
    cylinder_coeff.values[6] = 0.1;
    // viewer->addCylinder(cylinder_coeff);

    //do transformation
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    // z axis: (0, 0, 1)
    // trunk cylinder direction:
    //float x = cylinder_coeff.values[3];
    //float y = cylinder_coeff.values[4];
    //float z = cylinder_coeff.values[5];
    float x = -coefficients_cylinder_1->values[3];
    float y = -coefficients_cylinder_1->values[4];
    float z = -coefficients_cylinder_1->values[5];

    // calculate the cross product of z axis and trunk cylinder direction
    float x_norm = y;  //1*y - 0*z
    float y_norm = -x; //0*z - x
    float z_norm = 0;  //0*x - 0*y
    // calculate the unit vector of cross product vector
    float l = sqrt(pow(x_norm, 2) + pow(y_norm, 2));
    x_norm = x_norm / l;
    y_norm = y_norm / l;
    z_norm = z_norm / l;

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
    result = identity + std::sin(theta) * rotation + 2 * pow(std::sin(theta / 2), 2) * rotation * rotation;
    std::cout << "result" << std::endl;
    std::cout << result << std::endl;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            transform_1(i, j) = result(i, j);
        }
    }

    pcl::PointCloud<PointT>::Ptr cloud_new(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *cloud_new, transform_1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb_new(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_rgb, *cloud_rgb_new, transform_1);

    //transform the starting point of the cylinder model
    pcl::PointCloud<PointT>::Ptr cyl_start_point(new pcl::PointCloud<PointT>);
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
    pcl::PointCloud<PointT>::Ptr new_cyl_start_point(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cyl_start_point, *new_cyl_start_point, transform_1);
    std::cout << "cyl point transform: " << new_cyl_start_point->points[0] << std::endl;

    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_new, 0, 255, 0); // Red
    //viewer->addPointCloud (cloud_new, transformed_cloud_color_handler, "transformed_cloud");

    //elevate the point cloud
    float z_min = cloud_new->points[0].z;
    float z_max = cloud_new->points[0].z;
    for (int i = 0; i < cloud_new->points.size(); ++i)
    {
        if (cloud_new->points[i].z < z_min)
        {
            z_min = cloud_new->points[i].z;
        }
        if (cloud_new->points[i].z > z_max)
        {
            z_max = cloud_new->points[i].z;
        }
    }
    std::cout << "z range: " << z_min << " to " << z_max << std::endl;
    for (int i = 0; i < cloud_new->points.size(); ++i)
    {
        cloud_new->points[i].z -= z_min;
        cloud_rgb_new->points[i].z -= z_min;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler_org(cloud_new, 255, 140, 0); // Orange
    // viewer->addPointCloud(cloud_new, transformed_cloud_color_handler_org, "transformed_cloud_up");

    //Saved the final pcd file
    // writer.write("rotPlant_preprocessed.pcd", *cloud_new, false);

    //get PC of the stem
    std::cout << "1" << std::endl;
    pcl::ModelCoefficients::Ptr cylinder_coeff_tran(new pcl::ModelCoefficients);
    std::cout << "2" << std::endl;
    cylinder_coeff_tran->values.resize(7); // We need 7 values
    std::cout << "3" << std::endl;
    cylinder_coeff_tran->values[0] = new_cyl_start_point->points[0].x;
    cylinder_coeff_tran->values[1] = new_cyl_start_point->points[0].y;
    cylinder_coeff_tran->values[2] = new_cyl_start_point->points[0].z;
    cylinder_coeff_tran->values[3] = 0;
    cylinder_coeff_tran->values[4] = 0;
    cylinder_coeff_tran->values[5] = 1;
    cylinder_coeff_tran->values[6] = 0.01;

    std::cout << "before" << endl;
    // viewer->addCylinder(*cylinder_coeff_tran, "test");
    std::cout << "after" << endl;

    //pcl::PointCloud<PointT>::Ptr stem_cloud (new pcl::PointCloud<PointT>);
    //pcl::PointCloud<PointT>::Ptr rest_cloud (new pcl::PointCloud<PointT>);
    //getStemPC(cloud_new, stem_cloud, rest_cloud, cylinder_coeff_tran, OFFSET);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    getStemPC_RGB(cloud_rgb_new, stem_cloud, rest_cloud, cylinder_coeff_tran, OFFSET);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_2(new pcl::visualization::PCLVisualizer("3D Viewer 2"));
    viewer_2->setBackgroundColor(0, 0, 0);
    viewer_2->addCoordinateSystem(1.0);
    viewer_2->addCylinder(*cylinder_coeff_tran, "test");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> stem_cloud_handler (stem_cloud, 255, 0, 0); // Red
    //viewer_2->addPointCloud (stem_cloud, stem_cloud_handler, "stem_cloud");
    viewer_2->addPointCloud(stem_cloud, "stem_cloud");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rest_cloud_handler (rest_cloud, 0, 255, 0); // Green
    //viewer_2->addPointCloud (rest_cloud, rest_cloud_handler, "rest_cloud");
    viewer_2->addPointCloud(rest_cloud, "rest_cloud");
    std::cout << "size1: " << stem_cloud->size() << std::endl;
    std::cout << "size2: " << rest_cloud->size() << std::endl;

    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (layer_cloud, cloud_normals_1, 10, 0.05, "normals");

    // writer.write("leaf_cloud.pcd", *stem_cloud, false);
    // writer.write("stem_cloud.pcd", *rest_cloud, false);
    writer.write(leavesPcFn, *stem_cloud, false);
    writer.write(stemPcFn, *rest_cloud, false);

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }
    // return (0);
}
