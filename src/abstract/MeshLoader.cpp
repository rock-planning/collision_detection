#include "abstract/MeshLoader.hpp"
using namespace collision_detection;

MeshLoader::MeshLoader()
{
}

MeshLoader::~MeshLoader()
{
}

void MeshLoader::createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud,
                                          double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z,
                                          Eigen::Isometry3f target_frame)
{
    pcl::PointCloud<pcl::PointXYZ> verts;

    createPointCloudFromMesh(abs_path_to_mesh_file, verts,
                             scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z );

    pcl::transformPointCloud (verts, point_cloud, target_frame);
}
void MeshLoader::createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud, 
                                          double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z )
{

    Assimp::Importer assimp_importer;
    const aiScene * scene;
/*
 There are several way to read a cad file, available ways to read he cad file are:
    aiProcess_Triangulate |aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes
*/
    scene=assimp_importer.ReadFile(abs_path_to_mesh_file.c_str() , aiProcess_Triangulate); 

    LOG_DEBUG_S<<"[MeshLoader]: Extracting vertex and triangles from mesh file:" <<abs_path_to_mesh_file.c_str();
    LOG_DEBUG_S<<"[MeshLoader]: Number of meshes in the scene: " <<scene->mNumMeshes;


    pcl::PointXYZ pcl_point;

    for(std::size_t i=0;i<scene->mNumMeshes;i++ )
    {
        for(std::size_t j=0;j<scene->mMeshes[i]->mNumVertices;j++ )
        {
            pcl_point.x=scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x;
            pcl_point.y=scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y;
            pcl_point.z=scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z;
            point_cloud.push_back(pcl_point);
        }
    }
//    delete scene;

}
