#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP


#include "collision_detection/abstract/AbstractCollisionDetection.hpp"
#include "collision_detection/CollisionConfig.hpp"
#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"
#include "collision_detection/fcl_wrapper/CollisionData.hpp"


#include <string>
#include <vector>
#include <sys/stat.h> //needed to create director (mkdir function)

#include <fcl/config.h>
#if (FCL_MAJOR_VERSION > 0 || (FCL_MAJOR_VERSION >= 0 && \
     (FCL_MINOR_VERSION > 4 || (FCL_MINOR_VERSION >= 4 && \
      FCL_PATCH_VERSION > 0))))
#include <memory>
#include <functional>
#else
#define USE_BOOST_SHARED_PTR
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#endif

#include <fcl/fcl.h>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

namespace collision_detection
{

#ifdef USE_BOOST_SHARED_PTR
    using ::boost::shared_ptr;
    using ::boost::make_shared;
    using ::boost::dynamic_pointer_cast;
    using ::boost::static_pointer_cast;
    using ::boost::function1;
#else
    using ::std::shared_ptr;
    using ::std::make_shared;
    using ::std::dynamic_pointer_cast;
    using ::std::static_pointer_cast;
    template <class T, class U>
    using function1 = ::std::function<T(U)>;
#endif

typedef std::pair <std::string , shared_ptr<fcl::CollisionObject<double> > > CollisionObjectPair;
typedef std::multimap <std::string , shared_ptr<fcl::CollisionObject<double>> > CollisionObjectsMap;

class FCLCollisionDetection;

class FCLCollisionDetection: public AbstractCollisionDetection
{
    public:

//         FCLCollisionDetection(OctreeDebugConfig octree_debug_config_, bool use_contact_info=false);
        FCLCollisionDetection(CollisionDetectionConfig collision_detection_config);
        
        virtual ~FCLCollisionDetection();

        int numberOfObjectsInCollisionManger();

        void getCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &collision_manager);

        bool extractTrianglesAndVerticesFromMesh(const std::string &abs_path_to_mesh_file, std::vector<fcl::Triangle> &triangles, std::vector<fcl::Vector3d>& vertices, 
                         double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z );

        void registerOctreeToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap, const base::Pose &collision_object_pose, std::string link_name);

        void registerBoxToCollisionManager(const double &box_x, const double &box_y, const double &box_z, const std::string &link_name ,
                                                 const base::Pose &collision_object_pose, const double &link_padding = 1.20 );

        bool registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, const std::string &link_name, 
                        const base::Pose &collision_object_pose, const double &link_padding = 1.20);

        void registerMeshToCollisionManager(const std::string &link_name, const base::Pose &collision_object_pose, 
                        const std::vector<fcl::Triangle> &triangles, const std::vector<fcl::Vector3d> &vertices);   

        void registerCylinderToCollisionManager(const double &radius, const double &length, const std::string &link_name ,
                              const base::Pose &collision_object_pose, const double &link_padding = 1.20);

        void registerSphereToCollisionManager(const double &radius, const std::string &link_name, const base::Pose &collision_object_pose, const double &link_padding = 1.20); 

        void updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose);

        void updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap, const std::string &env_object_name);
        
        bool isCollisionsOccured( double &total_cost);
            
        bool checkEnvironmentCollision(const shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager, int num_max_contacts=1);

        bool assignWorldDetector(AbstractCollisionPtr collision_detector);

        bool removeSelfCollisionObject(const std::string &collision_object_name);
        
        bool removeWorldCollisionObject(const std::string &collision_object_name);
        
        void removeObject4mCollisionContainer(const std::string &collision_object_name);
        
        bool removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size);

        shared_ptr<fcl::BroadPhaseCollisionManager<double>>  &getCollisionManager();

        shared_ptr<fcl::BroadPhaseCollisionManager<double>> broad_phase_collision_manager;


        bool distanceOfClosestObstacleToRobot( shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager,
                           DistanceData &distance_data);


        std::vector< std::pair<std::string, std::string> > getCollidedObjectsNames();

        void printCollisionObject();

        std::vector< DistanceInformation>& getCollisionDistanceInformation();
        
        std::vector< DistanceInformation>& getCompleteDistanceInformation();

//         void computeSelfDistanceInfo();
// 
//         void computeClosestObstacleToRobotDistanceInfo();
// 
//         std::vector<DistanceInformation> &getClosestObstacleToRobotDistanceInfo();
        
        void saveOctree();

        std::vector<std::string> getRobotCollisionObjectsNames();

        std::vector<std::string> getWorldCollisionObjectsNames();

        int num_octree_;
        
    private:
        void registerCollisionObjectToCollisionManager(const std::string &link_name, shared_ptr< fcl::CollisionObject<double> > &collision_object );
        double getCollisionCost(CollisionData &collision_data, std::vector<DistanceInformation> &contacts);
        DistanceData getDistanceData();
        CollisionData getCollisionData();
        void calculateCompleteDistanceInfo();
        
        CollisionDetectionConfig collision_detection_config_;

        std::vector< std::pair<std::string, std::string> >  collision_object_names_;
        std::vector<  CollisionObjectAssociatedData *>  collision_data_;
        CollisionObjectsMap collision_objects_container_;
        Eigen::Vector3d scale_mesh_;
        std::shared_ptr<FCLCollisionDetection> world_collision_detector_;
        std::vector<DistanceInformation> env_collision_distance_information_;
        std::vector<DistanceInformation> full_collision_distance_information_;  // includes both self and environment collision information
        
        shared_ptr<octomap::OcTree > octomap_ptr_;
        shared_ptr< fcl::CollisionObject<double> > fcl_tree_collision_object_ptr_;

};
}// end namespace trajectory_optimization
#endif // COLLISIONDETECTION_HPP
