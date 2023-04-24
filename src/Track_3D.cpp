#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <iostream>
#include <typeinfo>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <ctime>
#include <pcl/impl/instantiate.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>  
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/PointIndices.h>
#include <pcl/impl/pcl_base.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <rviz_visual_tools/rviz_visual_tools.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/impl/integral_image_normal.hpp>
#include <pcl/search/search.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/common/transforms.h>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/range_image/range_image_planar.h>
//T0D0
//changed:
//increase attempts even when recovered by distance
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
typedef int frame; 
frame f = 0;
int errors = 0;
float max_voxel_size = 0.2f; // 0.3
int min_density = 10;         //40 completly different
int max_attempts = 20;
int voxel_id_reference=0;
bool tracking = false;
int track_id = 0;
bool track_recovered_by_distance = false;
visualization_msgs::MarkerArray centroids_array;
int track_dropped = 0;
int no_person_found =0;

struct OusterPoint
{
    PCL_ADD_POINT4D;
    float intensity;
    unsigned int t;
    unsigned short reflectivity;
    unsigned char ring;
    unsigned short ambient;
    unsigned int range;
    int voxel_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

pcl::PointCloud<OusterPoint>::Ptr cloud_reference (new pcl::PointCloud<OusterPoint>);

POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPoint,
                                    (float,x,x)
                                    (float,y,y)
                                    (float,z,z)
                                    (float,intensity,intensity)
                                    (unsigned int,t,t)
                                    (unsigned short,reflectivity,reflectivity)
                                    (unsigned char,ring,ring)
                                    (unsigned short,ambient,ambient)
                                    (unsigned int,range,range)

)

struct Voxel
{
    //id
    int voxel_id;
    //center:
    OusterPoint center;
    //dimensions:
    float length; // from x
    float width; // from y
    float height; //from z
    pcl::PointCloud<OusterPoint> member_points;
};

std::vector<Voxel> reference_voxel_list;

struct SuperVoxel
{
    Voxel voxel;
    float mean_intensity;
    float var_intensity;
    float mean_reflectivity;
    float var_reflectivity;
    float mean_ambient;
    float var_ambient;
    float mean_range;
    int object_id = -1;
    pcl::PointCloud<pcl::Normal> normals;
    Eigen::Vector4f normal;
};

struct Chain
{
    SuperVoxel principal_link;
    std::unordered_map<int, SuperVoxel> secondary_links;
    bool linked = false;
};

struct Object
{
    std::unordered_map<int,SuperVoxel> links;
    bool checked = false;
    bool joined = false;
    float mean_reflectivity;
    float mean_variance;
    float mean_intensity;
    float mean_ambient;
    float width;
    float length;
    float height;
    pcl::PointCloud<OusterPoint> cloud;
    Eigen::Vector4d centroid;
    float distance;
    float person_votes = 0.0f;
    float background_votes = 0.0f;
    float nx_ratio;
    float ny_ratio;
    float nz_ratio;
    float mean_range;
};

struct Track
{
    int id;
    std::map<frame, Eigen::Vector2d> shift_seq;
    std::map<frame, Object> object_seq; 
    std::map<frame, Eigen::Vector4d> tracked_centroid_seq;
    std::map<frame, bool> recovered_by_distance_seq; // this three maps store only the last 30 (3s) of data
    int attempt = 0;
};
std::map<int, Track> track_map;


bool operator==(const SuperVoxel& lhs, const SuperVoxel& rhs){
    return lhs.voxel.voxel_id == rhs.voxel.voxel_id;
}

bool operator==(const Eigen::Vector4d& lhs, const Eigen::Vector4d& rhs){
    return lhs(0) == rhs(0) &&
            lhs(1) == rhs(1) &&
            lhs(2) == rhs(2);
}

void initialize(pcl::PointCloud<OusterPoint>::Ptr cloud){
  for (auto& point: *cloud)
  {
    point.voxel_id = -1;
  }

}

Eigen::Vector4f normalize(Eigen::Vector4f normal){
    Eigen::Vector4f normalized(3,1);
    float length  = std::sqrt(normal(1,0) * normal(1,0) + normal(2,0) * normal(2,0) + normal(0,0) * normal(0,0));
    normalized(0,0) = normal(0,0) / (length);
    normalized(1,0) = normal(1,0) / (length);
    normalized(2,0) = normal(2,0) / (length);
    return normalized;
}

float dotProduct(Eigen::Vector4f v1, Eigen::Vector4f v2){
    return v1(1,0) * v2(1,0) + v1(2,0) * v2(2,0) + v1(0,0) * v2(0,0);
}

void display(pcl::PointCloud<OusterPoint>::Ptr cloud){
    for (auto& point: *cloud)
    {
        std::cout << "    " << point.x
                << " "    << point.y
                << " "    << point.z 
                // //   << " intensity: " << point.intensity
                // //   << " t: " << point.t
                // //   << " reflectivity: " << point.reflectivity
                << " ring: " << static_cast<int>(point.ring)
                // //   << " ambient: "<< point.ambient
                << " voxel: " <<point.voxel_id << std::endl;
    }
    std::cout << "size of cloud now: " << (*cloud).size() << std::endl;
}

void cloud_filter(pcl::PointCloud<OusterPoint>::Ptr cloud){
    if(cloud->height > 32){
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<OusterPoint> extract;
    for(unsigned int i = 0; i < (*cloud).size(); i++){
        if ((static_cast<int>(cloud->points[i].ring) % 2 != 0)){
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
}
}

void demeanCloud(pcl::PointCloud<OusterPoint>::Ptr cloud){
    Eigen::Vector4d centroid;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    compute3DCentroid(*cloud, centroid);
    //std::cout << "centroidddddd before " << centroid << std::endl;
    transform.translation() << -centroid(0), -centroid(1), -centroid(2);
    pcl::transformPointCloud(*cloud, *cloud, transform);
    // compute3DCentroid(*cloud, centroid);
    // std::cout << "centroidddddd after " << centroid << std::endl;
}

float getDistance(Eigen::Vector4d centroid1, Eigen::Vector4d centroid2){
    return std::sqrt( (centroid2(0) - centroid1(0))*(centroid2(0) - centroid1(0)) + (centroid2(1) - centroid1(1))*(centroid2(1) - centroid1(1)) );
}

void createVoxels(pcl::PointCloud<OusterPoint>::Ptr cloud, std::vector<Voxel> * voxel_list)
{
    pcl::KdTreeFLANN<OusterPoint> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;  
    float radius = max_voxel_size;
    int voxel_id = 0;
    for (auto& point: *cloud)
    {
       //if(point.range <= 2000){
        if (point.voxel_id == -1)
        {
            if (kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > min_density)
            {
                // create voxel if neighbours are found:
                Voxel voxel;
                voxel_id = voxel_id + 1;
                point.voxel_id = voxel_id;
                voxel.voxel_id = voxel_id;
                // assign the search point to be the center of the voxel;
                voxel.center = point;
                voxel.member_points.push_back(point);
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    (*cloud)[pointIdxRadiusSearch[i]].voxel_id = voxel_id;
                    voxel.member_points.push_back((*cloud)[pointIdxRadiusSearch[i]]);
                }
                (*voxel_list).push_back(voxel);
            }
        }
     // }
    }
}

void findNewVoxels(pcl::PointCloud<OusterPoint>::Ptr cloud, std::vector<Voxel> * new_voxel_list, bool update){
    pcl::KdTreeFLANN<OusterPoint> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;  
    float radius = max_voxel_size;
    kdtree.setInputCloud(cloud);
    for(auto& point: *cloud){
        if(point.voxel_id == -1){
            if (kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > min_density){
                std::vector<int> neighbor_index;
                for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    OusterPoint neighbor = (*cloud)[pointIdxRadiusSearch[i]];
                    if(neighbor.voxel_id == -1){
                        neighbor_index.push_back(pointIdxRadiusSearch[i]);
                    }
                } 
                if((signed int)neighbor_index.size() > min_density){
                    Voxel voxel;
                    voxel_id_reference +=1;
                    voxel.voxel_id = voxel_id_reference;
                    point.voxel_id = voxel_id_reference;
                    // assign the search point to be the center of the voxel;
                    voxel.center = point;
                    voxel.member_points.push_back(point);
                    for(auto idx: neighbor_index){
                        (*cloud)[idx].voxel_id = voxel_id_reference;
                        voxel.member_points.push_back((*cloud)[idx]);
                    }
                    if(update){
                        reference_voxel_list.push_back(voxel);
                    }else{
                        (*new_voxel_list).push_back(voxel);
                    }
                }
            }
        } 
    }
}

void createVoxelsMovement(pcl::PointCloud<OusterPoint>::Ptr cloud)
{
    pcl::KdTreeFLANN<OusterPoint> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;  
    float radius = max_voxel_size;
    kdtree.setInputCloud(cloud);
    if(reference_voxel_list.size() == 0){
        for (auto& point: *cloud)
        {
            if (point.voxel_id == -1)
            {
                if (kdtree.radiusSearch(point,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > min_density)
                {
                    // create voxel if neighbours are found:
                    Voxel voxel;
                    voxel_id_reference +=1;
                    point.voxel_id = voxel_id_reference;
                    voxel.voxel_id = voxel_id_reference;
                    // assign the search point to be the center of the voxel;
                    voxel.center = point;
                    voxel.member_points.push_back(point);
                    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                        (*cloud)[pointIdxRadiusSearch[i]].voxel_id = voxel_id_reference;
                        voxel.member_points.push_back((*cloud)[pointIdxRadiusSearch[i]]);
                    }
                    (reference_voxel_list).push_back(voxel);
                }
            }
        }
}else{
    for(auto voxel: reference_voxel_list){
        if(kdtree.radiusSearch(voxel.center,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance) > min_density){
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                (*cloud)[pointIdxRadiusSearch[i]].voxel_id = voxel.voxel_id;
                voxel.member_points.push_back((*cloud)[pointIdxRadiusSearch[i]]);
            }
        }
    }
}
}

void findObjectDimensions(std::unordered_map<int,Object> * object_list){
     //std::cout <<"step 1" << std::endl;
    for(auto& pairObject: *object_list){
        bool first = true;
        float smallest_x=0.0f;
        float smallest_y=0.0f;
        float smallest_z=0.0f;
        float largest_x=0.0f;
        float largest_y=0.0f;
        float largest_z=0.0f;
        for(auto pairSVoxel: pairObject.second.links){
            if(first){
                smallest_x = pairSVoxel.second.voxel.member_points[0].x;
                //std::cout <<"step 2" << std::endl;
                smallest_y = pairSVoxel.second.voxel.member_points[0].y;
                smallest_z = pairSVoxel.second.voxel.member_points[0].z;
                largest_x = pairSVoxel.second.voxel.member_points[0].x;
                largest_y = pairSVoxel.second.voxel.member_points[0].y;
                largest_z = pairSVoxel.second.voxel.member_points[0].z;
            }
            //std::cout <<"step 3" << std::endl;
            for(unsigned int i = 0; i < pairSVoxel.second.voxel.member_points.size(); i++){
                //std::cout <<"step 4" << std::endl;
                 //std::cout << i;
                pairObject.second.cloud.push_back(pairSVoxel.second.voxel.member_points[i]);
                if(pairSVoxel.second.voxel.member_points[i].x < smallest_x){
                    smallest_x = pairSVoxel.second.voxel.member_points[i].x;
                }
                if(pairSVoxel.second.voxel.member_points[i].y < smallest_y){
                    smallest_y = pairSVoxel.second.voxel.member_points[i].y;
                }
                if(pairSVoxel.second.voxel.member_points[i].z < smallest_z){
                    smallest_z = pairSVoxel.second.voxel.member_points[i].z;
                }
                if(pairSVoxel.second.voxel.member_points[i].x > largest_x){
                    largest_x = pairSVoxel.second.voxel.member_points[i].x;
                }
                if(pairSVoxel.second.voxel.member_points[i].y > largest_y){
                    largest_y = pairSVoxel.second.voxel.member_points[i].y;
                }
                if(pairSVoxel.second.voxel.member_points[i].z > largest_z){
                    largest_z = pairSVoxel.second.voxel.member_points[i].z;
                }

            }
            //std::cout <<"First for done" << std::endl;
            first = false;
        }
         //std::cout <<"second for done" << std::endl;
        pairObject.second.height = largest_z - smallest_z;
        pairObject.second.length = largest_x - smallest_x;
        pairObject.second.width = largest_y - smallest_y;  
        Eigen::Vector4d centroid;
        compute3DCentroid(pairObject.second.cloud, centroid);
        pairObject.second.centroid = centroid;
        pairObject.second.distance = getDistance(centroid, Eigen::Vector4d(0.0,0.0,0.0,0.0));
    }
}

void findVoxelsDimensions(std::vector<Voxel> * voxel_list){
    for(auto& voxel: *voxel_list){
        float smallest_x = voxel.member_points[0].x;
        float smallest_y = voxel.member_points[0].y;
        float smallest_z = voxel.member_points[0].z;
        float largest_x = voxel.member_points[0].x;
        float largest_y = voxel.member_points[0].y;
        float largest_z = voxel.member_points[0].z;
        for(unsigned int i = 1; i < voxel.member_points.size(); i++){
            if(voxel.member_points[i].x < smallest_x){
                smallest_x = voxel.member_points[i].x;
            }
            if(voxel.member_points[i].y < smallest_y){
                smallest_y = voxel.member_points[i].y;
            }
            if(voxel.member_points[i].z < smallest_z){
                smallest_z = voxel.member_points[i].z;
            }
            if(voxel.member_points[i].x > largest_x){
                largest_x = voxel.member_points[i].x;
            }
            if(voxel.member_points[i].y > largest_y){
                largest_y = voxel.member_points[i].y;
            }
            if(voxel.member_points[i].z > largest_z){
                largest_z = voxel.member_points[i].z;
            }
        }
        voxel.height = largest_z - smallest_z;
        voxel.length = largest_x - smallest_x;
        voxel.width = largest_y - smallest_y;
    }
}

visualization_msgs::Marker createDot(frame f, Eigen::Vector4d centroid ,float r, float g, float b, bool first){
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "os_sensor";
            sphere.header.stamp = ros::Time();
            sphere.ns = "centroids";
            sphere.id = f;
            sphere.type = visualization_msgs::Marker::SPHERE;
            if(first){
                sphere.action = visualization_msgs::Marker::DELETEALL;
            }else{
                sphere.action = visualization_msgs::Marker::ADD;
            }
            sphere.pose.position.x = centroid(0) ;
            sphere.pose.position.y = centroid(1) ;
            sphere.pose.position.z = centroid(2) ;
            sphere.pose.orientation.x = 0.0f;
            sphere.pose.orientation.y =0.0f;
            sphere.pose.orientation.z =0.0f;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.05f;
            sphere.scale.y = 0.05f;
            sphere.scale.z = 0.05f;
            sphere.color.r = r;
            sphere.color.g = g;
            sphere.color.b = b;
            sphere.color.a = 1.0; 
            sphere.lifetime = ros::Duration();
            return sphere;  
}

void visualizationCentroids(){
    if(!track_map.empty()){
        float r;
        float g;
        float b;
        for(auto trackPair: track_map){
            if(trackPair.first == 0){
                r = 0.0f;
                g = 1.0f;
                b = 0.0f;
            }else{
                r = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
                g = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
                b = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
            }
            int lastKeyCentroid = trackPair.second.tracked_centroid_seq.rbegin()->first;
            (centroids_array).markers.push_back(createDot(lastKeyCentroid, trackPair.second.tracked_centroid_seq[lastKeyCentroid] , r,g,b, false));
        }
    }
}

visualization_msgs::Marker createReference(){
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "os_sensor";
            arrow.header.stamp = ros::Time();
            arrow.ns = "voxelization";
            arrow.id = 9999999;
            arrow.type = visualization_msgs::Marker::ARROW;
            // if(first){
            //     arrow.action = visualization_msgs::Marker::DELETEALL;
            // }else{
            //     arrow.action = visualization_msgs::Marker::ADD;
            // }
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            p1.x = 0.0f;
            p1.y = 0.0f;
            p1.z = 0.0f;
            p2.x = 0.0f;
            p2.y = 0.0f;
            p2.z = 1.0f;
            arrow.points.push_back(p1);
            arrow.points.push_back(p2);
            arrow.scale.x = 0.15f;
            arrow.scale.y = 0.2f;
            arrow.scale.z = 0.0f;
            arrow.color.r = 1.0f;
            arrow.color.g = 0.0f;
            arrow.color.b = 0.0f;
            arrow.color.a = 1.0; 
            arrow.lifetime = ros::Duration();
            return arrow;
}

visualization_msgs::Marker createArrow(SuperVoxel svoxel, float r, float g, float b){
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "os_sensor";
            arrow.header.stamp = ros::Time();
            arrow.ns = "normals";
            arrow.id = svoxel.voxel.voxel_id;
            arrow.type = visualization_msgs::Marker::ARROW;
            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            // p1.x = svoxel.voxel.member_points[5].x;
            // p1.y = svoxel.voxel.member_points[5].y;
            // p1.z = svoxel.voxel.member_points[5].z;
            p1.x = svoxel.voxel.center.x;
            p1.y = svoxel.voxel.center.y;
            p1.z = svoxel.voxel.center.z;
            arrow.points.push_back(p1);
            // p2.x =svoxel.voxel.member_points[5].x + svoxel.normals[5].normal_x;
            // p2.y = svoxel.voxel.member_points[5].y + svoxel.normals[5].normal_y;
            // p2.z = svoxel.voxel.member_points[5].z + svoxel.normals[5].normal_z;
            p2.x = svoxel.voxel.center.x + svoxel.normal(0,0);
            p2.y = svoxel.voxel.center.y + svoxel.normal(1,0);
            p2.z = svoxel.voxel.center.z + svoxel.normal(2,0);
            arrow.points.push_back(p2);
            arrow.scale.x = 0.02f;
            arrow.scale.y = 0.03f;
            arrow.scale.z = 0.0f;
            arrow.color.r = r;
            arrow.color.g = g;
            arrow.color.b = b;
            arrow.color.a = 1.0; 
            arrow.lifetime = ros::Duration();
            return arrow;
}

visualization_msgs::Marker createMarker(SuperVoxel svoxel, float r, float g, float b, bool first){
            visualization_msgs::Marker cuboid;
            cuboid.header.frame_id = "os_sensor";
            cuboid.header.stamp = ros::Time();
            cuboid.ns = "voxelization";
            cuboid.id = svoxel.voxel.voxel_id;
            cuboid.type = visualization_msgs::Marker::CUBE;
            if(first){
                cuboid.action = visualization_msgs::Marker::DELETEALL;
            }else{
                cuboid.action = visualization_msgs::Marker::ADD;
            }
            cuboid.pose.position.x = svoxel.voxel.center.x ;
            cuboid.pose.position.y = svoxel.voxel.center.y ;
            cuboid.pose.position.z = svoxel.voxel.center.z ;
            cuboid.pose.orientation.x = 0.0f;
            cuboid.pose.orientation.y =0.0f;
            cuboid.pose.orientation.z =0.0f;
            cuboid.pose.orientation.w = 1.0;
            cuboid.scale.x = svoxel.voxel.length;
            cuboid.scale.y = svoxel.voxel.width;
            cuboid.scale.z = svoxel.voxel.height;
            cuboid.color.r = r;
            cuboid.color.g = g;
            cuboid.color.b = b;
            cuboid.color.a = 1.0; 
            cuboid.lifetime = ros::Duration();
            return cuboid;
}

void visualizationMarkers(std::unordered_map<int,Object> * object_list, visualization_msgs::MarkerArray * marker_array, bool show_normals){
   bool first = true;
    for(auto& pairObject: *object_list){
        // float r = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
        // float g = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
        // float b = static_cast<float>(rand()) / static_cast<float> (RAND_MAX);
        float r;
        float g;
        float b;
        if(pairObject.second.person_votes > pairObject.second.background_votes){
            r = 1.0f;
            g = 0.0f;
            b = 0.0f;
        }else{
            r = 0.0f;
            g = 0.0f;
            b = 1.0f;   
        }// }else{
        //     r = 1.0f;
        //     g = 1.0f;
        //     b = 1.0f;   
        // }
            for(auto const& pairSVoxel: pairObject.second.links){
                (*marker_array).markers.push_back(createMarker(pairSVoxel.second,r,g,b, first));
                if(show_normals)
                    (*marker_array).markers.push_back(createArrow(pairSVoxel.second,r,g,b));
                (*marker_array).markers.push_back(createReference());
                first = false;
            }
}
}

float getMean(std::vector<float> v){
    float sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    return sum / v.size();
}

float getVar(std::vector<float> v, float m){
    float accum = 0.0;
    std::for_each (std::begin(v), std::end(v), [&](const double r) {
        accum += (r - m) * (r - m);
    });
    return (accum / (v.size()-1));
}

pcl::PointCloud<pcl::Normal> getNormals(pcl::PointCloud<OusterPoint> member_points){
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<OusterPoint>::Ptr cloud (new pcl::PointCloud<OusterPoint> (member_points));

    pcl::NormalEstimation<OusterPoint, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<OusterPoint>::Ptr tree (new pcl::search::KdTree<OusterPoint> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch(max_voxel_size * 0.8);
    ne.compute(*normals);
    return (*normals);
}

Eigen::Vector4f getNormalCenter(pcl::PointCloud <OusterPoint> member_points, OusterPoint center){
    pcl::PointCloud<OusterPoint>::Ptr cloud (new pcl::PointCloud<OusterPoint> (member_points));
    //demeanCloud(cloud);
    pcl::KdTreeFLANN<OusterPoint> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;  
    float radius = max_voxel_size;
    kdtree.radiusSearch(center,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);
    Eigen::Vector4f normal_vector;
    float curvature;
    computePointNormal((*cloud),pointIdxRadiusSearch, normal_vector, curvature);
    return normalize(normal_vector);
}

void createSupervoxels(std::vector<Voxel> voxel_list, std::vector<SuperVoxel> * supervoxel_list, bool eliminateGround){
    for(auto voxel: voxel_list){
        SuperVoxel supervoxel;
        supervoxel.voxel = voxel;
        std::vector<float> intensity_vector;
        std::vector<float> reflectivity_vector;
        std::vector<float> ambient_vector;
        std::vector<float> range_vector;
        for(auto point: voxel.member_points){
            intensity_vector.push_back(point.intensity);
            reflectivity_vector.push_back((float)point.reflectivity);
            ambient_vector.push_back((float)point.ambient);
            range_vector.push_back((float)point.range);
        }
        supervoxel.mean_intensity = getMean(intensity_vector);
        supervoxel.mean_reflectivity = getMean(reflectivity_vector);
        supervoxel.mean_ambient = getMean(ambient_vector);
        supervoxel.mean_range = getMean(range_vector)/1000.0f;
        supervoxel.var_intensity = getVar(intensity_vector,supervoxel.mean_intensity);
        supervoxel.var_reflectivity = getVar(reflectivity_vector,supervoxel.mean_reflectivity);
        supervoxel.var_ambient = getVar(ambient_vector,supervoxel.mean_ambient);
       //supervoxel.normals = getNormals(voxel.member_points);
        supervoxel.normal = getNormalCenter(voxel.member_points, voxel.center);
        //eliminate ground
        if(eliminateGround){
            Eigen::Vector4f normalGround;
            normalGround(0,0) = 0.0f;
            normalGround(1,0) = 0.0f;
            normalGround(2,0) = 1.0f;
            float angle = acos(dotProduct(normalGround, supervoxel.normal)) * (180 / 3.14159) ;
            // std::cout << "normal calculated " << supervoxel.normal << std::endl;
            if((angle > 20)){
                (*supervoxel_list).push_back(supervoxel);
                // std::cout << "########### angle " << angle << std::endl;
            }
        }else{
            (*supervoxel_list).push_back(supervoxel);
        }
    }
}

bool checkDistanceCondition(SuperVoxel svoxel_p, SuperVoxel svoxel_n){
    float cd = 0;
    bool distance = false;
        if (!(svoxel_p == svoxel_n)){
            if((std::abs(svoxel_p.voxel.center.x - svoxel_n.voxel.center.x) <= (svoxel_p.voxel.length + svoxel_n.voxel.length)/2 +cd)){
                if(std::abs(svoxel_p.voxel.center.y - svoxel_n.voxel.center.y) <= (svoxel_p.voxel.width + svoxel_n.voxel.width)/2 + cd ){
                    if((std::abs(svoxel_p.voxel.center.z - svoxel_n.voxel.center.z) <= (svoxel_p.voxel.height + svoxel_n.voxel.height)/2  + cd)){
                        distance = true;
                    }                
                }                
            }
        }
    return distance;
}

bool checkIntensityCondition(SuperVoxel svoxel_p, SuperVoxel svoxel_n){
    if(std::abs(svoxel_p.mean_intensity - svoxel_n.mean_intensity) <= 3 * std::sqrt(std::max(svoxel_p.var_intensity, svoxel_n.var_intensity))){
        return true;
    }else{
        return false;
    }
}

bool checkReflectivityCondition(SuperVoxel svoxel_p, SuperVoxel svoxel_n){
    if(std::abs(svoxel_p.mean_reflectivity - svoxel_n.mean_reflectivity) <= 3 * std::sqrt(std::max(svoxel_p.var_reflectivity, svoxel_n.var_reflectivity))){
        return true;
    }else{
        return false;
    }
}

bool checkAmbientCondition(SuperVoxel svoxel_p, SuperVoxel svoxel_n){
    if(std::abs(svoxel_p.mean_ambient - svoxel_n.mean_ambient) <= 3 * std::sqrt(std::max(svoxel_p.var_ambient, svoxel_n.var_ambient))){
        return true;
    }else{
        return false;
    }
}

void findSecondaryLinks(std::vector<SuperVoxel> supervoxel_list, Chain * chain){
    SuperVoxel svoxel_p = (*chain).principal_link;
    for(auto svoxel_n: supervoxel_list){
        if(checkDistanceCondition(svoxel_p, svoxel_n)){
            if(checkIntensityCondition(svoxel_p,svoxel_n)){
                if(checkReflectivityCondition(svoxel_p,svoxel_n)){
                    if(checkAmbientCondition(svoxel_p,svoxel_n)){
                        (*chain).secondary_links[svoxel_n.voxel.voxel_id] = svoxel_n;
                    }
                }
            }
        }
    }
}

void findChains(std::vector<SuperVoxel> supervoxel_list, std::vector<Chain> * chain_list){
    for(auto svoxel: supervoxel_list){
        Chain chain;
        chain.principal_link = svoxel;
        findSecondaryLinks(supervoxel_list, &chain);
        if(chain.secondary_links.size() > 0){
            (*chain_list).push_back(chain);
        }
    }
}

void segmentObjects(std::vector<Chain> * chain_list, std::unordered_map<int,Object> * object_list){
    int i = -1;
    for(auto& chain_r: *chain_list){
        if(!chain_r.linked){
            std::unordered_map<int,SuperVoxel> links_local = chain_r.secondary_links;
            for(auto& chain_tbf: *chain_list){
                if(chain_tbf.secondary_links.find(chain_r.principal_link.voxel.voxel_id) != chain_tbf.secondary_links.end()){
                    chain_tbf.linked = true;
                    links_local.insert(chain_tbf.secondary_links.begin(),chain_tbf.secondary_links.end());
                } 
            } 
            i+=1;
            Object o;
            o.links= links_local;
            (*object_list)[i] = o;
    }
    }
}

std::vector<int> getIntersection (std::unordered_map<int,SuperVoxel> links_r, std::unordered_map<int,SuperVoxel> links_tbf){
    std::vector<int> intersection;
    for(auto pair: links_r){
        if(links_tbf.find(pair.first) != links_tbf.end()){
            intersection.push_back(pair.first);
        }
    }
    return intersection;
}

void joinRedundantObjects(std::unordered_map<int,Object> * object_list, std::unordered_map<int,Object> * object_list_filtered){
    for(auto& pair_r : *object_list){
        if(!pair_r.second.checked){
            for(auto& pair_tbf : *object_list){
            if((!pair_tbf.second.joined) & (!pair_tbf.second.checked) & (pair_r.first != pair_tbf.first)){ 
                if(getIntersection(pair_r.second.links,pair_tbf.second.links).size() > 0){
                    pair_r.second.links.insert(pair_tbf.second.links.begin(),pair_tbf.second.links.end());
                    pair_tbf.second.checked = true;
                }
            }
            }
            pair_r.second.joined = true;
    }
    }
    //std::cout <<"number objects before filtering " << (*object_list).size() << std::endl;
    for(auto& pair: *object_list){
        if((pair.second.joined) & (pair.second.links.size() > 2)){
            (*object_list_filtered)[pair.first] = pair.second;
        }
    }
}

unsigned int displayObjectInfo(std::unordered_map<int,Object> object_list_filtered){
//   std::cout << "number of objects #### " << object_list_filtered.size() << std::endl;
//   std::cout << " with #links: "<< std::endl;
  unsigned int object_vox_count = 0;
  for(auto o: object_list_filtered){
      object_vox_count += o.second.links.size();
    //   std::cout <<o.second.links.size() << " id " << o.first <<std::endl;
  }
  return object_vox_count;
}

void checkAgain(std::unordered_map<int,Object> * object_list_filtered){
     std::unordered_set<int> ind_delete;
        for(auto& pair_r : *object_list_filtered){
            for(auto& pair_tbf : *object_list_filtered){
                if(pair_r.first != pair_tbf.first){
                    if(getIntersection(pair_r.second.links,pair_tbf.second.links).size() > 0){
                        if(pair_r.second.links.size() >= pair_tbf.second.links.size()){
                            ind_delete.insert(pair_tbf.first);
                        }
                    }
            }
            }
        }
        // std::cout <<"ind delete ";
        // std::cout <<" size "<< ind_delete.size()<<std::endl;
        for(auto i: ind_delete){
            (*object_list_filtered).erase(i);
        }
}

bool checkError(unsigned int object_vox_count, unsigned int total_voxels, bool again){
    if(object_vox_count != total_voxels){
        if(!again){
            errors +=1;
        }
        //std::cout << " mistake in segmentation"  <<std::endl;
        return true;
  }else{
      if(again){
          errors -=1;
      }
      return false;
  }
}

void trySolveError(std::unordered_map<int,Object> * object_list_filtered, unsigned int total_vox){
    // std::cout << " solve error ........... " << std::endl;
    checkAgain(object_list_filtered);
    unsigned int object_vox_count = displayObjectInfo(*object_list_filtered);
    if(!checkError(object_vox_count, total_vox, true)){
        // std::cout << " error solved " <<std::endl;
}
}

void findObjectAtt(std::unordered_map<int,Object> * object_list_filtered){
    for(auto& pair: *object_list_filtered){
        float normal_x = 0.0f;
        float normal_y = 0.0f;
        float normal_z = 0.0f;
        float total_normal = 0.0f;
        std::vector<float> object_reflectivity;
        std::vector<float> object_intensity;
        std::vector<float> object_ambient;
        std::vector<float> object_variance;
        std::vector <float> object_range;
        for(auto pair_sv: pair.second.links){
            object_reflectivity.push_back(pair_sv.second.mean_reflectivity);
            object_intensity.push_back(pair_sv.second.mean_intensity);
            object_ambient.push_back(pair_sv.second.mean_ambient);
            object_range.push_back(pair_sv.second.mean_range);
            normal_x += std::abs(pair_sv.second.normal(0,0));
            normal_y += std::abs(pair_sv.second.normal(1,0));
            normal_z += std::abs(pair_sv.second.normal(2,0));
        }
        pair.second.mean_reflectivity = getMean(object_reflectivity);
        pair.second.mean_intensity = getMean(object_intensity);
        pair.second.mean_ambient = getMean(object_ambient);
        pair.second.mean_range = getMean(object_range);
        total_normal = normal_x + normal_y + normal_z;
        pair.second.nx_ratio = normal_x / total_normal;
        pair.second.ny_ratio = normal_y / total_normal;
        pair.second.nz_ratio = normal_z / total_normal;
    }
}

void shapeClassifier(std::unordered_map<int,Object> * object_list){
    for(auto& pairObject: *object_list){
        if( (pairObject.second.width > pairObject.second.height) | (pairObject.second.length > pairObject.second.height) | ( (pairObject.second.width > 1.0f) & (pairObject.second.length > 0.7f) ) | (pairObject.second.width < 0.28f) | (pairObject.second.length < 0.28f) ){
            pairObject.second.background_votes += 1.0f;
        }else if((pairObject.second.height > 1.2f) & (pairObject.second.height < 2.2f) & (pairObject.second.width < 1.8f) & (pairObject.second.length < 1.8f) & ((pairObject.second.width > 0.4f) | (pairObject.second.length > 0.4f)) ){
                pairObject.second.person_votes += 1.0f;
        }else{
            pairObject.second.background_votes +=1.0f;
        }
    }
}

void normalClassifier(std::unordered_map<int,Object> * object_list){
    float parallel_th = 0.72f;
    for(auto& pairObject: *object_list){
        if( (pairObject.second.nx_ratio > parallel_th) | (pairObject.second.ny_ratio > parallel_th) ){
            pairObject.second.background_votes += 1.0f;
        }
    }
}

void shadowClassifier(std::unordered_map<int,Object> * object_list){
    for(auto& pairObject: *object_list){
        if(pairObject.second.person_votes >= pairObject.second.background_votes){
            for(auto& pairObjectCmp: *object_list){
                if(pairObjectCmp.second.person_votes >= pairObjectCmp.second.background_votes){
                    if(pairObject.first != pairObjectCmp.first){
                        if(pairObject.second.distance < pairObjectCmp.second.distance){
                            float separation = pairObjectCmp.second.distance - pairObject.second.distance;
                            float x_separation = std::abs(pairObjectCmp.second.centroid(0) - pairObject.second.centroid(0)); 
                            //float y_separation = std::abs(pairObjectCmp.second.centroid(1) - pairObject.second.centroid(1)); 
                            if((x_separation >= 0.90 * separation)){
                                pairObjectCmp.second.background_votes +=1;
                            }
                        }
                    }
                }
            }
        }
    }
}

void makeOutputCloud(std::unordered_map<int,Object> * object_list, pcl::PointCloud<OusterPoint> * cloud_output, pcl::PointCloud<OusterPoint>::Ptr cloud_complete, pcl::PointCloud<OusterPoint>::Ptr cloudROI, bool detected_person){
    bool found_someone_distance = false;
    bool found = false;
    for(auto& point: *cloud_complete){
        point.ring = 2;
        (*cloud_output).push_back(point);
    }
        OusterPoint p;
        p.x = 1.0f;
        p.y = 1.0f;
        p.z = 1.0f;
        p.ring = 32;
        (*cloud_output).push_back(p);
        OusterPoint p2;
        p2.x = -1.0f;
        p2.y = -1.0f;
        p2.z = -1.0f;
        p2.ring = 64;
        (*cloud_output).push_back(p2);
        for(auto& pairObject: *object_list){
            if(detected_person){
                for(auto point: pairObject.second.cloud){
                    point.ring = 64;
                    (*cloud_output).push_back(point);
                    found = true;
                }
            }else{
                if(pairObject.second.person_votes > pairObject.second.background_votes){
                    for(auto point: pairObject.second.cloud){
                        point.ring = 64;
                        (*cloud_output).push_back(point);
                        found = true;
                    } 
                    found_someone_distance = true;
                }
            }
        }
        if((!found_someone_distance) & (!tracking)){
            for(auto point: *cloudROI){
                point.ring = 32;
                (*cloud_output).push_back(point);
            }
        }
        if(!found){
            no_person_found+=1;
        }

}

void makeOutputs(std::unordered_map<int,Object> * object_list, std::unordered_map<int,Object> detected_person,visualization_msgs::MarkerArray * marker_array, pcl::PointCloud<OusterPoint> * cloud_output ,pcl::PointCloud<OusterPoint>::Ptr cloudComplete, pcl::PointCloud<OusterPoint>::Ptr cloudROI,bool show_normals){
  bool detected_person_bool = false;
  if(!tracking){
    visualizationMarkers(object_list, marker_array, show_normals);
    //make output cloud;
    makeOutputCloud(object_list, cloud_output, cloudComplete, cloudROI, detected_person_bool);
  }else{
    if(detected_person.size() > 0){
        detected_person_bool = true;
        visualizationMarkers(&detected_person, marker_array, show_normals);
        makeOutputCloud(&detected_person, cloud_output, cloudComplete, cloudROI, detected_person_bool);
        track_map[0].attempt = 0; 
        // if(!track_recovered_by_distance){
        //     track_map[0].attempt = 0; 
        // }else{
        // track_map[0].attempt+=1;
        // }

    }else{
        track_map[0].attempt+=1;
        visualizationMarkers(object_list, marker_array, show_normals);
        makeOutputCloud(object_list, cloud_output, cloudComplete, cloudROI, detected_person_bool);
    }
  }
   visualizationCentroids();
}

void fillDetectedPerson(std::unordered_map<int,Object> object_list, std::unordered_map<int,Object> * detected_person){
    for(auto pairObject: object_list){
        if(pairObject.second.person_votes > pairObject.second.background_votes){
            (*detected_person)[pairObject.first] = pairObject.second;
        }
    }
}

Eigen::Vector4d getPredictedCentroid(){ // should the prediction consider the lqst centroid or the lqst clqssified centroid?
    Eigen::Vector4d predicted_centroid;
    //int lastKey = track_map[0].object_seq.rbegin()->first;
    int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
    predicted_centroid(0) = track_map[0].tracked_centroid_seq[lastKeyCentroid](0) + track_map[0].shift_seq[lastKeyCentroid](0);
    predicted_centroid(1) = track_map[0].tracked_centroid_seq[lastKeyCentroid](1) + track_map[0].shift_seq[lastKeyCentroid](1);
    predicted_centroid(2) = track_map[0].tracked_centroid_seq[lastKeyCentroid](2);
    predicted_centroid(3) = track_map[0].tracked_centroid_seq[lastKeyCentroid](3);
    return predicted_centroid;
}

void roiCloud(pcl::PointCloud<OusterPoint>::Ptr cloudIn, pcl::PointCloud<OusterPoint>::Ptr cloudOut){
    float baseDiff = 0.025f;
    float dx = (track_map[0].attempt + 1) * baseDiff;
    float dy = (track_map[0].attempt + 1) * baseDiff;
    float max_length = 1.2f;
    float max_width = 1.2f;
    Eigen::Vector4d predicted_centroid;
    int lastKey = track_map[0].object_seq.rbegin()->first;
    int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
    if((track_map[0].shift_seq[lastKey](0) == -99)){
        predicted_centroid = track_map[0].tracked_centroid_seq[lastKeyCentroid];
        dx+= baseDiff;
        dy+= baseDiff;
    }else{
        predicted_centroid = getPredictedCentroid();
    }
    pcl::PointCloud<OusterPoint> cloudOut_n;
    for(auto& point: *cloudIn){
        if( (point.x >= predicted_centroid(0) - dx - max_length) & (point.x <= predicted_centroid(0) + dx + max_length) ){
            if( (point.y >= predicted_centroid(1) - dy - max_width) & (point.y <= predicted_centroid(1) + dy + max_width) ){
                cloudOut_n.push_back(point);
            }
        }
    }
    *cloudOut = cloudOut_n;
}

void getVoxelCloud(pcl::PointCloud<OusterPoint>::Ptr cloud_reference){
    for(auto voxel: reference_voxel_list){
        cloud_reference->push_back(voxel.center);
    }

}

void getVoxelCloudROI(pcl::PointCloud<OusterPoint>::Ptr cloud_roi, std::vector<Voxel> new_voxel_list){
    for(auto voxel: new_voxel_list){
        for(auto point: voxel.member_points){
            cloud_roi->push_back(point);
        }
    }

}

void observationsInGatingRegion(std::unordered_map<int,Object> * object_list, std::unordered_map<int,Object> *detected_person){
    float radiusGating = 0.3f;
    float dx = radiusGating;
    float dy = radiusGating;
    int lastKey = track_map[0].object_seq.rbegin()->first;
    int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
    for (auto pairObject: *object_list){
        Eigen::Vector4d predicted_centroid;
        if((track_map[0].shift_seq[lastKey](0) == -99)){
            predicted_centroid = track_map[0].tracked_centroid_seq[lastKeyCentroid];
            dx += radiusGating;
            dy += radiusGating;
        }else{
            predicted_centroid = getPredictedCentroid();
        }
        if( (pairObject.second.centroid(0) >= predicted_centroid(0) - dx) & (pairObject.second.centroid(0) <= predicted_centroid(0) + dx) & (pairObject.second.centroid(1) >= predicted_centroid(1) - dy) & (pairObject.second.centroid(1) <= predicted_centroid(1) + dy)){
            (*detected_person)[pairObject.first] = pairObject.second;
        }
    }
}

void createTrack(Eigen::Vector2d shift, Object o){
    Track tr;
    tr.id = track_id;
    tr.shift_seq[f] = shift;
    tr.object_seq[f] = o;
    tr.recovered_by_distance_seq[f] = track_recovered_by_distance;
    if(o.person_votes > o.background_votes){
        tr.tracked_centroid_seq[f] = o.centroid;
    }
    track_map[tr.id] = tr;
}

void reduceToOnePerson(std::unordered_map<int,Object> * detected_person){
      float min_distance = 99.0f;
      Eigen::Vector4d min_distance_centroid(0,0,0,0);
      int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
      for (auto pairPerson: *detected_person){
          float distance = getDistance(track_map[0].tracked_centroid_seq[lastKeyCentroid], pairPerson.second.centroid);
          if(distance < min_distance){
              min_distance = distance;
              min_distance_centroid(0) = pairPerson.second.centroid(0);
              min_distance_centroid(1) = pairPerson.second.centroid(1);
              min_distance_centroid(2) = pairPerson.second.centroid(2);
              min_distance_centroid(3) = pairPerson.second.centroid(3);
          }
      } 
        std::unordered_set<int> ind_delete;
        for(auto pairPerson: *detected_person){
            // if its not the closest two last or normals say its wall
            if( (!(pairPerson.second.centroid == min_distance_centroid)) | (min_distance > 0.5f) ){
                pairPerson.second.person_votes -= 1.0f;
                ind_delete.insert(pairPerson.first);
            }
        }
        for(auto i: ind_delete){
            (*detected_person).erase(i);
        }
}

void filterByClassInfo(std::unordered_map<int,Object> * detected_person){
    float parallel_th = 0.72f;
    std::unordered_set<int> ind_delete;
    for(auto& pairObject: *detected_person){
        if( (pairObject.second.nx_ratio > parallel_th) | (pairObject.second.ny_ratio > parallel_th) | (pairObject.second.height >= 2.2f) | (pairObject.second.width >= 2.0f) | (pairObject.second.length >= 2.0f) | ((pairObject.second.width < 0.4f) & (pairObject.second.length < 0.4f)) ){
            ind_delete.insert(pairObject.first);
        }
    }
    for(auto i: ind_delete){
        (*detected_person).erase(i);
    }
}

void updateTracks(std::unordered_map<int,Object> * detected_person){
            int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
            int lastKey = track_map[0].object_seq.rbegin()->first;
            for(auto pairPerson: *detected_person){ 
                float shift_x;
                float shift_y;
                float real_pred_distance = -99;
                float expected_pred_distance= 0;
                if((track_recovered_by_distance) | (track_map[0].recovered_by_distance_seq[lastKey]) ){
                    shift_x = -99;
                    shift_y = -99;
                }else{
                    real_pred_distance = getDistance(getPredictedCentroid(), pairPerson.second.centroid);
                    expected_pred_distance = getDistance(getPredictedCentroid(), track_map[0].tracked_centroid_seq[lastKeyCentroid]);
                    shift_x = pairPerson.second.centroid(0) - track_map[0].tracked_centroid_seq[lastKeyCentroid](0);
                    shift_y = pairPerson.second.centroid(1) - track_map[0].tracked_centroid_seq[lastKeyCentroid](1);
                }
                    if(track_map[0].object_seq.size() == 30){
                        int firstKey = track_map[0].object_seq.begin()->first;
                        track_map[0].object_seq.erase(firstKey);
                        track_map[0].shift_seq.erase(firstKey);
                        track_map[0].recovered_by_distance_seq.erase(firstKey);
                    }
                    track_map[0].object_seq[f] = pairPerson.second;
                    track_map[0].shift_seq[f] = Eigen::Vector2d(shift_x,shift_y);
                    track_map[0].recovered_by_distance_seq[f] = track_recovered_by_distance;
                    float pred_distance_error = std::abs(expected_pred_distance - real_pred_distance);
                    if((pairPerson.second.person_votes > pairPerson.second.background_votes) & (pred_distance_error < 0.5f)){
                        track_map[0].tracked_centroid_seq[f] = pairPerson.second.centroid;
                    }
            }    
}

void handleTracks(std::unordered_map<int,Object> * detected_person){
    if(detected_person->size() > 1){
        reduceToOnePerson(detected_person); // change this when multiple objects, reduces to one person taking into account only distance 
    }
    if(track_recovered_by_distance){
        filterByClassInfo(detected_person); // filter by normals
    }
    if(detected_person->size() > 0){
        if(track_map.size() == 0){ // create track if no track exist
            for(auto pairPerson: *detected_person){
                createTrack(Eigen::Vector2d(-99, -99), pairPerson.second);
                //std::cout << " ############################################# tracked created" << std::endl;
            }   
        }else{ // update track if track exists
            updateTracks(detected_person);
        }
    }
}

void showObjects(std::unordered_map<int,Object> object_list){
  for(auto& pair: object_list){
      std::cout << " object id "
      << pair.first
      << " l, w, h: " << pair.second.length << " , " << pair.second.width << " , " << pair.second.height 
      << " d  : "<<pair.second.distance << " mean range: " << pair.second.mean_range 
      << " centroid x, y , z " << pair.second.centroid(0) << " , " << pair.second.centroid(1) << " , " << pair.second.centroid(2) 
      << " nx, ny, nz: "<<pair.second.nx_ratio <<" , " <<pair.second.ny_ratio << " , "<<pair.second.nz_ratio 
      //<< " shift is " <<track_map[0].shift_seq[f](0) << " , " << track_map[0].shift_seq[f](1) 
      << std::endl; 
  }
}

void movementDetection(pcl::PointCloud<OusterPoint>::Ptr cloudComplete, pcl::PointCloud<OusterPoint>::Ptr cloudReference, std::vector<Voxel> * new_voxel_list){
  createVoxelsMovement(cloudComplete);
  bool update = false;
  if((f < 10) & (f > 5)){ // should be f>3
    // calibration of reference
    update = true;
    findNewVoxels(cloudComplete, new_voxel_list ,update);
    getVoxelCloud(cloudReference);
  }else{
      if((f > 5)){ // f>3
          findNewVoxels(cloudComplete, new_voxel_list, update);
          getVoxelCloudROI(cloudReference, *new_voxel_list);
      }
  }
  //std::cout <<" voxel count " << new_voxel_list->size() << std::endl;
}

void fillReferenceVoxelList(std::unordered_map<int,Object> detected_person, std::vector<Voxel> voxel_list){
      std::unordered_map<int, Voxel> reference_voxel_map;
      for(auto vox: voxel_list){
          reference_voxel_map[vox.voxel_id] = vox;
      }
      if(detected_person.size() > 0){
          std::unordered_set<int> id_delete;
          for(auto pairPerson: detected_person){
              for(auto sVoxelPair: pairPerson.second.links){
                  id_delete.insert(sVoxelPair.second.voxel.voxel_id);
              }
          }
          for(auto id: id_delete){
              reference_voxel_map.erase(id);
          }
      }
      for(auto pairVox: reference_voxel_map){
          reference_voxel_list.push_back(pairVox.second);
      }
}

void findVoxelList(pcl::PointCloud<OusterPoint>::Ptr cloudComplete, pcl::PointCloud<OusterPoint>::Ptr cloudROI, std::vector<Voxel> * voxel_list){
  if (track_map.size() == 0){
    if(f <= 5){ // should be 1
        //std::cout << " ########### detection by classification" << std::endl;
        cloudROI = cloudComplete;
        createVoxels(cloudROI,voxel_list);
        tracking = false;
    }else{
        //std::cout << " ########### movement detection" << std::endl;
        movementDetection(cloudComplete,cloudROI, voxel_list);
        tracking = false;
    }
  }else{
      if(track_map[0].attempt < max_attempts){
        //std::cout << "tracking attempt " << track_map[0].attempt << std::endl; 
        roiCloud(cloudComplete, cloudROI);
        createVoxels(cloudROI, voxel_list);
        tracking = true; 
      }else{
          //std::cout << "end tracking" << std::endl; 
          track_map[0].attempt = 0;
          track_map.erase(0); // change when multiple
          track_dropped+=1;
          movementDetection(cloudComplete,cloudROI, voxel_list);
          tracking = false;
      }
  }
}

void recoverByDistance(std::unordered_map<int,Object> * detected_person,std::unordered_map<int,Object> * object_list){
  //std::cout << "track not classified, trying to recover by distance .. " << std::endl;
  observationsInGatingRegion(object_list, detected_person);
  track_recovered_by_distance = true;
  handleTracks(detected_person); 
  if(detected_person->size() > 0){
    //std::cout << "track recovered by distance" <<std::endl;
    track_recovered_by_distance = true;
  }else{
      //std::cout << "track not recovered" << std::endl; // maybe here increase the attempts? not sure if its is here
      track_recovered_by_distance = true;
  }
}

void trackOps(std::unordered_map<int,Object> * object_list, std::unordered_map<int,Object> * detected_person, std::vector<Voxel> voxel_list){
  if(detected_person->size() > 0){ // if someone was found by classification
    if (f > 1){
        track_recovered_by_distance = false;
        handleTracks(detected_person);
    }else{
        if(f == 1){
            fillReferenceVoxelList(*detected_person, voxel_list);
            if(detected_person->size() == 1){ // change when multiple
                for(auto pairPerson: *detected_person){
                    createTrack(Eigen::Vector2d(-99,-99), pairPerson.second);
                  //  std::cout << " #################################################### track created " << std::endl;
                }
            }else{
                //std::cout << " ######################################################################################################### problem in first frame: more than one person detected failed " << std::endl;
            }
        }
    }
    //showObjects(*detected_person); // show only when someone found by classification
}else{ // if no one was found by classification but we are tracking then we can attempt to recover
    if(tracking){
        recoverByDistance(detected_person, object_list);
    }
}
// if(!track_map.empty()){
//     int lastKeyCentroid = track_map[0].tracked_centroid_seq.rbegin()->first;
//     std::cout << "new tracked center " <<track_map[0].tracked_centroid_seq[lastKeyCentroid](0) << " , " <<track_map[0].tracked_centroid_seq[lastKeyCentroid](1) << std::endl;
// }
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  std::cout << "frame start: " << f << std::endl;  
  // conversion:
  pcl::PointCloud<OusterPoint>::Ptr cloudComplete (new pcl::PointCloud<OusterPoint>);
  pcl::fromROSMsg (*input, *cloudComplete);
  initialize(cloudComplete);
  cloud_filter(cloudComplete);
  demeanCloud(cloudComplete);
  pcl::PointCloud<OusterPoint>::Ptr cloudROI (new pcl::PointCloud<OusterPoint>);
  std::vector<Voxel> voxel_list;
  // tracking
  findVoxelList(cloudComplete, cloudROI, &voxel_list);
  //
  findVoxelsDimensions(&voxel_list);
  std::vector<SuperVoxel> supervoxel_list;
  bool eliminateGround = true;
  createSupervoxels(voxel_list, &supervoxel_list, eliminateGround);
  std::vector<Chain> chain_list;
  findChains(supervoxel_list, &chain_list);
  //std::cout << "number of chains " << chain_list.size() << std::endl;
  std::unordered_map<int,Object> object_list;
  segmentObjects(&chain_list, &object_list);
  std::unordered_map<int,Object> object_list_filtered;
  joinRedundantObjects(&object_list, &object_list_filtered);
  unsigned int object_vox_count = displayObjectInfo(object_list_filtered);
  if(checkError(object_vox_count, chain_list.size(), false)){
      trySolveError(&object_list_filtered, chain_list.size());
    }  
  findObjectAtt(&object_list_filtered);
  findObjectDimensions(&object_list_filtered);
  //std::cout <<" number of objects in roi " << object_list_filtered.size() << std::endl;
  shapeClassifier(&object_list_filtered);
  normalClassifier(&object_list_filtered);
  shadowClassifier(&object_list_filtered);
  std::unordered_map<int,Object> detected_person;
  fillDetectedPerson(object_list_filtered, &detected_person);
  //std::cout << "nb detected person with classification "<< detected_person.size() << std::endl;
  //// tracking 2
  trackOps(&object_list_filtered, &detected_person, voxel_list);
  /////
  visualization_msgs::MarkerArray marker_array;
  pcl::PointCloud<OusterPoint> cloud_output;
  bool show_normals = false;
  makeOutputs(&object_list_filtered, detected_person, &marker_array, &cloud_output, cloudComplete ,cloudROI,show_normals);
  //Publish the data.
    ros::Rate loop_rate(10);
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 cloud_roi;
    pcl::toROSMsg(*cloudROI,cloud_roi); 
    pcl::toROSMsg(cloud_output, output);
    output.header = input->header;
    cloud_roi.header = input->header;
    pub.publish(marker_array);
    pub2.publish(cloud_roi);
    pub3.publish(output);
    pub4.publish(centroids_array);
    f = f + 1;
    ros::spinOnce();
    loop_rate.sleep();

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "listener");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud

    pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array",1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("roi", 1);
    pub4 = nh.advertise<visualization_msgs::MarkerArray>("centroids", 1);

  // Spin
  ros::spin ();
  std::cout << "track dropps "<< track_dropped <<std::endl;
  std::cout << "no found "<< no_person_found <<std::endl;
}
