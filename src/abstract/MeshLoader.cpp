#include <abstract/MeshLoader.hpp>
using namespace trajectory_optimization;

MeshLoader::MeshLoader()
{
}


void MeshLoader::extractTrianglesAndVerticesFromMesh(std::string &abs_path_to_mesh_file, std::vector<fcl::Triangle> &triangles, std::vector<fcl::Vector3d>& vertices, double scale_for_mesha_files_x=1.00, double scale_for_mesha_files_y=1.00, double scale_for_mesha_files_z=1.00 )
{
//    #define PRINT_LOG_ASSIMP
    Assimp::Importer assimp_importer;
    const aiScene * scene;


/*
 There are several way to read a cad file, available ways to read he cad file are:
    aiProcess_Triangulate |aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes
*/

    scene=assimp_importer.ReadFile(abs_path_to_mesh_file.c_str() , aiProcess_Triangulate);


    #ifdef PRINT_LOG_ASSIMP
    std::cout<<"start extracting vertex and triangles from mesh file:" <<abs_path_to_mesh_file.c_str() <<std::endl;
    std::cout<<"number of meshes in the scene: " <<scene->mNumMeshes<<std::endl;
    #endif


    fcl::Vector3d vetex;
    fcl::Triangle triangle;


    for(std::size_t i=0;i<scene->mNumMeshes;i++ )
    {
        #ifdef PRINT_LOG_ASSIMP
        std::cout<<"number of face in the mesh number "<< i <<" is: " <<scene->mMeshes[i]->mNumFaces <<std::endl;
        std::cout<<"reading (face) triangles indexes:" <<std::endl;
        #endif
        for(std::size_t j=0;j<scene->mMeshes[i]->mNumFaces;j++)
        {
            triangle.set(scene->mMeshes[i]->mFaces[j].mIndices[0], scene->mMeshes[i]->mFaces[j].mIndices[1] , scene->mMeshes[i]->mFaces[j].mIndices[2]);
            triangles.push_back(triangle);
        }
        #ifdef PRINT_LOG_ASSIMP
        std::cout<<"number of verticies (x,y,z poses) in the mesh number "<< i <<" is: " <<scene->mMeshes[i]->mNumVertices<<std::endl;
        std::cout<<"reading verticies: " <<std::endl;
        #endif
        for(std::size_t j=0;j<scene->mMeshes[i]->mNumVertices;j++ )
        {
            //vetex.setValue(scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x, scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y, scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z) ;
            vetex.x() = scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x;
            vetex.y() = scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y;
            vetex.z() = scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z;

            vertices.push_back(vetex);

        }
    }


//    delete scene;

}

void MeshLoader::createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud,
                                          double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z,
                                          Eigen::Affine3f target_frame)
{
    pcl::PointCloud<pcl::PointXYZ> verts;

    createPointCloudFromMesh(abs_path_to_mesh_file, verts,
                             scale_for_mesha_files_x, scale_for_mesha_files_y, scale_for_mesha_files_z );

    pcl::transformPointCloud (verts, point_cloud, target_frame);
}
void MeshLoader::createPointCloudFromMesh(std::string &abs_path_to_mesh_file, pcl::PointCloud<pcl::PointXYZ> &point_cloud, double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z )
{
//    #define PRINT_LOG_ASSIMP
    Assimp::Importer assimp_importer;
    const aiScene * scene;


/*
 There are several way to read a cad file, available ways to read he cad file are:
    aiProcess_Triangulate |aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes
*/

    scene=assimp_importer.ReadFile(abs_path_to_mesh_file.c_str() , aiProcess_Triangulate);


    #ifdef PRINT_LOG_ASSIMP
    std::cout<<"start extracting vertex and triangles from mesh file:" <<abs_path_to_mesh_file.c_str() <<std::endl;
    std::cout<<"number of meshes in the scene: " <<scene->mNumMeshes<<std::endl;
    #endif



    pcl::PointXYZ pcl_point;




    for(std::size_t i=0;i<scene->mNumMeshes;i++ )
    {

        #ifdef PRINT_LOG_ASSIMP
        std::cout<<"number of verticies (x,y,z poses) in the mesh number "<< i <<" is: " <<scene->mMeshes[i]->mNumVertices<<std::endl;
        std::cout<<"reading verticies: " <<std::endl;
        #endif
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
