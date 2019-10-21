#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

#define PI 3.14159265
#define WIDTH 100
#define HEIGHT 100
#define RADIUS 10
#define NOISE 0.1

int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = WIDTH;
  cloud.height   = HEIGHT;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);


  float angle_height = PI/cloud.height;
  float angle_width = 2*PI/cloud.width;

  std::cout << angle_height << std::endl;
  std::cout << angle_width << std::endl;

  for (size_t i = 0; i < cloud.width; i++) {
      for (size_t j = 0; j < cloud.height; j++) {
          cloud.points[i*cloud.height + j].x = sin(angle_height*j)*RADIUS*cos(angle_width*i); 
          cloud.points[i*cloud.height + j].y = sin(angle_height*j)*RADIUS*sin(angle_width*i);
          cloud.points[i*cloud.height + j].z = cos(angle_height*j)*RADIUS;
      }
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
