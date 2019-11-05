
typedef pcl::PointXYZ PointT;

float getPoint2LineDistance(float x,
                            float y,
                            float z,
                            float line_org_x,
                            float line_org_y,
                            float line_org_z,
                            float line_dir_x,
                            float line_dir_y,
                            float line_dir_z) {
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

void getStemPC(pcl::PointCloud<PointT>::Ptr org_cloud,
               pcl::PointCloud<PointT>::Ptr stem_cloud,
               pcl::PointCloud<PointT>::Ptr rest_cloud,
               pcl::ModelCoefficients::Ptr coefficients_cylinder,
               float offset_ratio) {
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
                                  coefficients_cylinder->values[5]) > (coefficients_cylinder->values[6] * offset_ratio)){
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

void getStemPC_RGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr org_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr rest_cloud,
                   pcl::ModelCoefficients::Ptr coefficients_cylinder,
                   float offset_ratio) {
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
                                  coefficients_cylinder->values[5]) > (coefficients_cylinder->values[6] * offset_ratio)){
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

// Shift the cloud to center
void shiftPC2Center(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb) {
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
    tree_height = 0;
    for (int i = 0; i < cloud->points.size(); ++i){
        if (cloud->points[i].x < x_min){
                x_min = cloud->points[i].x;
        }
        if (cloud->points[i].x > x_max){
                x_max = cloud->points[i].x;
        }
        if (cloud->points[i].y < y_min){
                y_min = cloud->points[i].y;
        }
        if (cloud->points[i].y > y_max){
                y_max = cloud->points[i].y;
        }
        if (cloud->points[i].z < z_min){
                z_min = cloud->points[i].z;
        }
        if (cloud->points[i].z > z_max){
                z_max = cloud->points[i].z;
        }
    }
    tree_height = z_max - z_min;
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
        cloud_rgb->points[i].z = cloud_rgb->points[i].z - z_min;
    }
}

pcl::PointCloud<PointT>::Ptr getRoughStemPC(pcl::PointCloud<PointT>::Ptr cloud, float radius) {
    pcl::PointCloud<PointT>::Ptr rough_stem_cloud (new pcl::PointCloud<PointT>);
    std::vector<float> x;
    std::vector<float> y; 
    for (int i = 0; i < cloud->points.size(); i++) {
        if (cloud->points[i].z < 0.25) {
            x.push_back(cloud->points[i].x);
            y.push_back(cloud->points[i].y);
        }
    }

    // calculate average of x and y vector
    float sum_x = 0;
    float sum_y = 0;
    int size = x.size();
    for (int i = 0; i < size; i++) {
        sum_x += x.at(i);
        sum_y += y.at(i);
    }
    float x_avg = sum_x/size;
    float y_avg = sum_y/size;
    std::cout << "x avg: " << x_avg << std::endl;
    std::cout << "y avg: " << y_avg << std::endl;

    // cut out the rough steam PC
    int counter = 0;
    for (int i = 0; i < cloud->points.size(); i++) {
        if ((pow(cloud->points[i].x - x_avg, 2) + pow(cloud->points[i].y - y_avg, 2)) < pow(radius, 2)) {
            rough_stem_cloud->points.push_back(cloud->points[i]);
            counter++;
        }
    }
    rough_stem_cloud->width = counter;
    rough_stem_cloud->height = 1;
    rough_stem_cloud->points.resize(counter);
    std::cout << "Counter: " << counter << std::endl;

    return rough_stem_cloud;
}

