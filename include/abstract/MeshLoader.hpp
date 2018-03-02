#ifndef MESHLOADER_HPP
#define MESHLOADER_HPP

#include <string>
#include <vector>

//#include <fcl/data_types.h>
#include <fcl/math/triangle.h>
//#include <fcl/math/vec_3f.h>
#include <fcl/common/types.h>
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>



#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

namespace collision_detection
{

class MeshLoader
{
public:
    MeshLoader();
    void extractTrianglesAndVerticesFromMesh(const std::string &abs_path_to_mesh_file, std::vector<fcl::Triangle> &triangles, std::vector<fcl::Vector3d>& vertices, double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z );
    void createPointCloudFromMesh(  std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud,
                                    double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z,
                                    Eigen::Affine3f target_frame);
    void createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud, double scale_for_mesha_files_x=1.00, double scale_for_mesha_files_y=1.00, double scale_for_mesha_files_z=1.00 );
};
}// end namespace trajectory_optimization
#endif // MESHLOADER_HPP
