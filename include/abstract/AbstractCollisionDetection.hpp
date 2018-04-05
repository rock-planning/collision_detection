#ifndef ABSTRACTCOLLISIONDETECTION_HPP_
#define ABSTRACTCOLLISIONDETECTION_HPP_

#include <vector>
#include <string>
#include <base/samples/RigidBodyState.hpp>
#include <srdfdom/model.h>
#include <pcl/point_cloud.h>
#include <octomap/octomap.h>

 #include <boost/bind.hpp>
 #include <base-logging/Logging.hpp>


/** \file AbstractCollisionDetection.hpp
*    \brief Abstract collision detection header.
*/

namespace collision_detection
{

enum CollisionLibrary
{
    FCL
};

struct DistanceInformation
{
    std::string object1;
    std::string object2;
    double distance;
};

class AbstractCollisionDetection;

typedef std::shared_ptr<AbstractCollisionDetection> AbstractCollisionPtr;

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

    virtual void registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, const std::string &link_name, 
						const base::Pose &collision_object_pose, const double &link_padding) = 0;
    
    virtual void registerBoxToCollisionManager(const double &box_x, const double &box_y, const double &box_z, const std::string &link_name,
                                             const base::Pose &collision_object_pose, const double &link_padding ) = 0;
    
    virtual void registerCylinderToCollisionManager(const double &radius, const double &length, const std::string &link_name ,
						  const base::Pose &collision_object_pose ,const double &link_padding) = 0;

    virtual void registerSphereToCollisionManager(const double &radius, const std::string &link_name, const base::Pose &collision_object_pose, const double &link_padding) = 0;

    virtual void updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose) = 0;
    
    virtual void removeSelfCollisionObject(const std::string &collision_object_name) = 0;
    
    virtual void removeWorldCollisionObject(const std::string &collision_object_name) = 0;
    
    virtual int numberOfObjectsInCollisionManger() = 0;
    
    virtual bool checkSelfCollision(int num_max_contacts=1) = 0;

    virtual bool checkWorldCollision(int num_max_contacts=1) = 0;    
   
    virtual bool assignWorldDetector(AbstractCollisionPtr collision_detector) = 0;
    
    virtual void printCollisionObject() = 0;
    
    virtual void printWorldCollisionObject() = 0;
    
    static bool linksToBeChecked( std::string first_link_name, std::string second_object_name );
    
    static std::vector<srdf::Model::DisabledCollision> disabled_collisions_;

    void setDisabledCollisionPairs(std::vector<srdf::Model::DisabledCollision> &disabled_collisions);

    void addDisabledCollisionPairs(srdf::Model::DisabledCollision &disabled_collision);

    void removeDisabledCollisionLink(const std::string &link);
    
    bool isLinkListed(srdf::Model::DisabledCollision const &remove_link);
       
    std::string remove_link_;
};

std::vector<srdf::Model::DisabledCollision> AbstractCollisionDetection::disabled_collisions_;
    
};

#endif

