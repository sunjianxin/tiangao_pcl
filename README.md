# tiangao_pcl

This is a C++ project for automatic feature extraction from point clouds of plant structure from University of Nebraska-Lincoln

Requirement: The input point cloud must be rotated such that the steam is pointing to the positive z axis

Build
```
#cd findcylinder_2
#mkdir build
#cd build
#cmake ..
#make
```
Run
findcylinder r_lower r_upper cut_offset_ratio radius input_pcd_file

r_lower: lower bound of the radius of the fitted cylinder model
r_upper: upper bound of the radius of the fitted cylinder model
cut_offset_ratio: all points that have a greater distance to the cylinder centroid axis than the fittec cylinder model radius multiply this cut_offset_ratio will be classified as leaf points, rest will be classified as steam points
radius: used for cutting out the intial point cloud used for fitting cylinder model, this is under the condition that the input point cloud is rotated such that the stem is pointing to the positive z axis
```
#./findcylinder_v2/build/findcylinder 0.1 0.2 2 2 data/xxx.pcd
```
