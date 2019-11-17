#ifndef ABSTRACTCOLLISIONDETECTION_HPP_
#define ABSTRACTCOLLISIONDETECTION_HPP_

#include <vector>
#include <string>
#include <base/samples/RigidBodyState.hpp>
#include <srdfdom/model.h>
#include <octomap/octomap.h>

#include <boost/bind.hpp>
#include <base-logging/Logging.hpp>


/** \file AbstractCollisionDetection.hpp
*   \brief Abstract collision detection header.
*/

namespace collision_detection
{

class AbstractCollisionDetection;

typedef std::shared_ptr<AbstractCollisionDetection> AbstractCollisionPtr;

struct DistanceInformation
{
    DistanceInformation(){nearest_points.resize(2);}
    std::string object1;
    std::string object2;
    double min_distance;
    std::vector< Eigen::Vector3d > nearest_points;
    Eigen::Vector3d contact_normal;
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

        virtual void registerOctreeToCollisionManager(  const std::shared_ptr<octomap::OcTree> &octomap, const base::Pose &collision_object_pose, 
                                                        std::string link_name) = 0;

        virtual bool registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, const std::string &link_name, 
                                                    const base::Pose &collision_object_pose, const double &link_padding) = 0;

        virtual void registerBoxToCollisionManager( const double &box_x, const double &box_y, const double &box_z, const std::string &link_name,
                                                    const base::Pose &collision_object_pose, const double &link_padding ) = 0;

        virtual void registerCylinderToCollisionManager(const double &radius, const double &length, const std::string &link_name ,
                                                        const base::Pose &collision_object_pose ,const double &link_padding) = 0;

        virtual void registerSphereToCollisionManager(const double &radius, const std::string &link_name, const base::Pose &collision_object_pose, const double &link_padding) = 0;

        virtual void updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose) = 0;

        virtual void updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap, const std::string &env_object_name) = 0;

        virtual bool removeSelfCollisionObject(const std::string &collision_object_name) = 0;

        virtual bool removeWorldCollisionObject(const std::string &collision_object_name) = 0;

        virtual bool removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size) = 0;

        virtual int numberOfObjectsInCollisionManger() = 0;

//         virtual bool checkSelfCollision(int num_max_contacts=1) = 0;
// 
//         virtual bool checkWorldCollision(int num_max_contacts=1) = 0;

        virtual bool isCollisionsOccured(double &collision_cost) = 0;

        virtual bool assignWorldDetector(AbstractCollisionPtr collision_detector) = 0;

        virtual void printCollisionObject() = 0;
        
        virtual void saveOctree() = 0;

        virtual std::vector< std::pair<std::string, std::string> > getCollisionObjectNames()=0;

        static bool linksToBeChecked( const std::string &first_link_name, const std::string &second_link_name )
        {
            if(first_link_name == second_link_name  )            
                return false;            

            //for(std::size_t i = 0; i < AbstractCollisionDetection::disabled_collisions_.size(); i++ )
            for(auto &dc: AbstractCollisionDetection::disabled_collisions_)
            {
                if( (first_link_name== dc.link1_ &&  second_link_name  == dc.link2_) || 
                    (second_link_name == dc.link1_ &&  first_link_name == dc.link2_)  )
                    return false;
            }
            return true;
        }

        static std::vector<srdf::Model::DisabledCollision> disabled_collisions_;

        void setDisabledCollisionPairs(std::vector<srdf::Model::DisabledCollision> &disabled_collisions);

        void addDisabledCollisionPairs(srdf::Model::DisabledCollision &disabled_collision);

        void removeDisabledCollisionLink(const std::string &link);

        bool isLinkListed(srdf::Model::DisabledCollision const &remove_link);

//         virtual void computeSelfDistanceInfo() = 0;

//         virtual void computeClosestObstacleToRobotDistanceInfo() = 0;

        virtual std::vector< DistanceInformation>& getCollisionDistanceInformation() = 0;

//         virtual std::vector<DistanceInformation> &getClosestObstacleToRobotDistanceInfo() = 0;

//         virtual std::vector<DistanceInformation> &getSelfContacts() = 0;
// 
//         virtual std::vector<DistanceInformation> &getEnvironmentalContacts() = 0;

        std::string remove_link_;
};


};

#endif

