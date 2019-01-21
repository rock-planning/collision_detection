#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP


#include "abstract/AbstractCollisionDetection.hpp"

#include "CollisionObjectAssociatedData.hpp"
#include "CollisionData.hpp"


#include <string>
#include <vector>

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

typedef std::pair <std::string , shared_ptr<fcl::CollisionObject<double> > > collision_objects_pair;
typedef std::multimap <std::string , shared_ptr<fcl::CollisionObject<double>> > collision_objects_maps;

class FCLCollisionDetection;

class FCLCollisionDetection: public AbstractCollisionDetection
{

private:
    void registerCollisionObjectToCollisionManager(const std::string &link_name, shared_ptr< fcl::CollisionObject<double> > &collision_object );
  

    std::vector<  CollisionObjectAssociatedData *>  collision_data_;
    collision_objects_maps collision_objects_container_;
    std::vector<DistanceInformation> self_collision_contacts_;
    std::vector<DistanceInformation> environment_collision_contacts_;
    Eigen::Vector3d scale_mesh_;    
    std::shared_ptr<FCLCollisionDetection> world_collision_detector_;
    //collision_detection::AbstractCollisionPtr world_collision_detector_;

    std::vector<DistanceInformation> self_collision_distance;
    std::vector<DistanceInformation> obstacle_collision_distance;
    
    shared_ptr<octomap::OcTree > octomap_ptr_;

    void fclContactToDistanceInfo(const std::vector<fcl::Contact<double> > &collision_contacts, std::vector<DistanceInformation> &contacts);

    DistanceData getDistanceData();

public:

    FCLCollisionDetection();
    
    virtual ~FCLCollisionDetection();

    int numberOfObjectsInCollisionManger();

    void getCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &collision_manager);

    void extractTrianglesAndVerticesFromMesh(const std::string &abs_path_to_mesh_file, std::vector<fcl::Triangle> &triangles, std::vector<fcl::Vector3d>& vertices, 
					 double scale_for_mesha_files_x, double scale_for_mesha_files_y, double scale_for_mesha_files_z );
    
    void registerOctreeToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap, const base::Position &sensor_origin,
                                            const base::Pose &collision_object_pose, std::string link_name);
    
    void registerPointCloudToCollisionManager( const pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud, const base::Position &sensor_origin,
					       const base::Pose &collision_object_pose,double octree_resolution, std::string link_name);    
    
    void registerBoxToCollisionManager(const double &box_x, const double &box_y, const double &box_z, const std::string &link_name ,
                                             const base::Pose &collision_object_pose, const double &link_padding = 1.0 );
   
    void registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, const std::string &link_name, 
					const base::Pose &collision_object_pose, const double &link_padding = 1.0);
    
    void registerMeshToCollisionManager(const std::string &link_name, const base::Pose &collision_object_pose, 
					const std::vector<fcl::Triangle> &triangles, const std::vector<fcl::Vector3d> &vertices);   

    void registerCylinderToCollisionManager(const double &radius, const double &length, const std::string &link_name ,
						  const base::Pose &collision_object_pose, const double &link_padding = 1.0);

    void registerSphereToCollisionManager(const double &radius, const std::string &link_name, const base::Pose &collision_object_pose, const double &link_padding = 1.0); 
    
    void updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose);
    
    void updateEnvironment(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud, const base::Position &sensor_origin, const std::string &env_object_name);
           
    void updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap, const base::Position &sensor_origin, const std::string &env_object_name);


    bool checkSelfCollision(int num_max_contacts=1);

    bool checkWorldCollision(int num_max_contacts=1);
        
    bool checkEnvironmentCollision(const shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager, int num_max_contacts=1);

    bool assignWorldDetector(AbstractCollisionPtr collision_detector);

    void removeSelfCollisionObject(const std::string &collision_object_name);
    
    void removeWorldCollisionObject(const std::string &collision_object_name);
    
    void removeObject4mCollisionContainer(const std::string &collision_object_name);

    shared_ptr<fcl::BroadPhaseCollisionManager<double>>  &getCollisionManager();

    shared_ptr<fcl::BroadPhaseCollisionManager<double>> broad_phase_collision_manager;

    /**
     * @brief Conversion from a PCL pointcloud to octomap::Pointcloud, used internally in OctoMap
     *
     * @param pclCloud
     * @param octomapCloud
     */
    
    void pointcloudPCLToOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pclCloud, octomap::Pointcloud& octomapCloud);    

    bool distanceOfClosestObstacleToRobot( shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager,
					   DistanceData &distance_data);

    std::vector <fcl::CollisionObject<double>*> getEnvironmentCollisionObject();

    std::vector < std::pair<fcl::CollisionObject<double>*,fcl::CollisionObject<double>* > > &getSelfCollisionObject();

    std::vector< std::pair<std::string, std::string> > getCollisionObjectNames();

    void printCollisionObject();       

    std::vector<DistanceInformation> &getSelfContacts();

    std::vector<DistanceInformation> &getEnvironmentalContacts();

    std::vector< DistanceInformation> &getSelfDistanceInfo();

    void computeSelfDistanceInfo();

    void computeClosestObstacleToRobotDistanceInfo();

    std::vector<DistanceInformation> &getClosestObstacleToRobotDistanceInfo();

};
}// end namespace trajectory_optimization
#endif // COLLISIONDETECTION_HPP
