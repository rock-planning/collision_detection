#ifndef ABSTRACTCOLLISIONDETECTION_HPP_
#define ABSTRACTCOLLISIONDETECTION_HPP_

#include <vector>
#include <string>
#include <base/samples/RigidBodyState.hpp>
#include <srdfdom/model.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>

 #include <boost/bind.hpp>


/** \file AbstractCollisionDetection.hpp
*    \brief Abstract collision detection header.
*/

namespace trajectory_optimization
{

enum CollisionLibrary
{
    FCL
};

/**
 * @class AbstractCollisionDetection
 * @brief Provides an abstract interface for collision detection.
 */
class AbstractCollisionDetection
{

public:
    /**
    * @brief  constructor
    */
    AbstractCollisionDetection();
    /**
    * @brief  destructor
    */
    virtual ~AbstractCollisionDetection();

    virtual void registerOctreeToCollisionManager ( octomap::OcTree octomap_octree, std::string link_name , const base::samples::RigidBodyState &collision_object) = 0;
    
    template <class PointT>
    void registerPointCloudToCollisionManager(const pcl::PointCloud<PointT>& pclCloud,  double octree_resolution, std::string link_name , const base::samples::RigidBodyState &collision_object) ;

    virtual void registerMeshToCollisionManager(std::string abs_path_to_mesh_file, Eigen::Vector3d mesh_scale, std::string link_name, const base::samples::RigidBodyState &collision_object, const double link_padding) = 0;

    virtual void updateCollisionObjectTransform(std::string link_name, const base::samples::RigidBodyState collision_objec) = 0;
    
    static bool linksToBeChecked( std::string first_link_name, std::string second_object_name );
    
    static std::vector<srdf::Model::DisabledCollision> disabled_collisions_;

private:
    void setDisabledCollisionPairs(std::vector<srdf::Model::DisabledCollision> &disabled_collisions);

    void addDisabledCollisionPairs(srdf::Model::DisabledCollision &disabled_collision);

    void removeDisabledCollisionLink(const std::string &link);
    
    bool isLinkListed(srdf::Model::DisabledCollision const &remove_link);
    

    

    

    std::string remove_link_;
};

};

#endif

