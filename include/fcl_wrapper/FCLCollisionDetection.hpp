#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP


#include <abstract/AbstractCollisionDetection.hpp>
#include <abstract/MeshLoader.hpp>

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
/*
#include <fcl/collision.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/BV.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/distance.h>
#include <fcl/octree.h>
#include <fcl/broadphase/broadphase.h>
*/

namespace trajectory_optimization
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


class DistanceInformation
{
public:
    std::string object1;
    std::string object2;
    double distance;

};

class CollisionDetection: public AbstractCollisionDetection
{

private:
    typedef std::pair <std::string , shared_ptr<fcl::CollisionObject<double> > > link_name_CollisionObject_entry;
    typedef std::multimap <std::string , shared_ptr<fcl::CollisionObject<double>> > link_names_CollisionObjects;
    MeshLoader mesh_loader;
    std::vector<  CollisionObjectAssociatedData *>  vector_of_CollisionObjectAssociatedData_addresses;
    link_names_CollisionObjects link_names_CollisionObjects_Container;
    std::vector<fcl::Contact<double>> self_collision_contacts;
    std::vector<fcl::Contact<double>> collision_contacts_against_external_collision_manager;

public:

    CollisionDetection();
    ~CollisionDetection();

    std::vector<fcl::Contact<double>> &getSelfContacts();
    std::vector<fcl::Contact<double>> &getContactsAgainstExternalCollisionManager();


    int numberOfObjectsInCollisionManger();

    void getCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &collision_manager);


    void registerBoxToCollisionObjectManager(const double x, const double y, const double z, std::string link_name ,
                                             const fcl::Quaterniond collision_object_quaternion_orientation,const fcl::Vector3d collision_object_translation ,const double link_padding );


    void registerMeshToCollisionObjectManager(double &scale_for_mesha_files_x, double &scale_for_mesha_files_y ,
			    double &scale_for_mesha_files_z  , std::string &link_name ,const fcl::Quaterniond &collision_object_quaternion_orientation,
			    const fcl::Vector3d &collision_object_translation ,std::vector<fcl::Triangle> &triangles, std::vector<fcl::Vector3d> &vertices);
    
    
    void registerMeshToCollisionObjectManager(std::string abs_path_to_mesh_file, double scale_for_mesha_files_x, double scale_for_mesha_files_y ,double scale_for_mesha_files_z  , std::string link_name ,const fcl::Quaterniond collision_object_quaternion_orientation,
                                              const fcl::Vector3d collision_object_translation ,const double link_padding);


    void registerCylinderToCollisionObjectManager(const double radius, const double length, std::string link_name ,
                                             const fcl::Quaterniond collision_object_quaternion_orientation,const fcl::Vector3d collision_object_translation ,const double link_padding );


    void registerSphereToCollisionObjectManager(const double radius, std::string link_name , const fcl::Quaterniond collision_object_quaternion_orientation
                                                ,const fcl::Vector3d collision_object_translation ,const double link_padding );




    template <class PointT>
    void registerPCLPointCloudToCollisionObjectManager(const pcl::PointCloud<PointT>& pclCloud,  double octree_resolution, std::string link_name , const fcl::Quaterniond collision_object_quaternion_orientation ,const fcl::Vector3d collision_object_translation )
    {
    //1)    conver pcl::pointcloud to octomap tree

        octomap::Pointcloud octomapCloud;
        this->pointcloudPCLToOctomap<PointT>( pclCloud, octomapCloud);
        octomap::point3d origin (0.0f, 0.0f, 0.0f);
        shared_ptr<octomap::OcTree > octomap_Octree_ptr(new octomap::OcTree(octree_resolution) );
        octomap_Octree_ptr->insertPointCloud(octomapCloud, origin);

    //2)    register octomap tree to fcl



        //    fcl::OcTree* fcl_tree = new fcl::OcTree(shared_ptr<const octomap::OcTree>(generateOcTree()));
        //    fcl::CollisionObject tree_obj((shared_ptr<fcl::CollisionGeometry>(fcl_tree)));

        shared_ptr<fcl::OcTree<double>> fcl_OcTree_ptr(new fcl::OcTree<double>(  octomap_Octree_ptr) ) ;

        //fcl::Transform3d fcl_tree_transform3d(collision_object_quaternion_orientation,collision_object_translation);
        //shared_ptr< fcl::CollisionObject<double>   > fcl_tree_collision_object_ptr (new fcl::CollisionObject<double>( fcl_OcTree_ptr, fcl_tree_transform3f) ) ;
        shared_ptr< fcl::CollisionObject<double>   > fcl_tree_collision_object_ptr (new fcl::CollisionObject<double>( fcl_OcTree_ptr) ) ;


        CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
        collision_object_associated_data->setID(link_name);
        fcl_tree_collision_object_ptr->setUserData( collision_object_associated_data );
        vector_of_CollisionObjectAssociatedData_addresses.push_back(collision_object_associated_data);

        broad_phase_collision_manager->registerObject(fcl_tree_collision_object_ptr.get());

        link_name_CollisionObject_entry link_name_CollisionObject;
        link_name_CollisionObject.first=link_name;
        link_name_CollisionObject.second=fcl_tree_collision_object_ptr;
        link_names_CollisionObjects_Container.insert(link_name_CollisionObject );

    }




    void registerOctomapOctreeToCollisionObjectManager(  octomap::OcTree octomap_octree, std::string link_name , const fcl::Quaterniond collision_object_quaternion_orientation ,const fcl::Vector3d collision_object_translation )
    {
    //1)    conver pcl::pointcloud to octomap tree


//        shared_ptr<octomap::OcTree > octomap_Octree_ptr(new octomap::OcTree(octree_resolution) );


    //2)    register octomap tree to fcl



        //    fcl::OcTree* fcl_tree = new fcl::OcTree(shared_ptr<const octomap::OcTree>(generateOcTree()));
        //    fcl::CollisionObject tree_obj((shared_ptr<fcl::CollisionGeometry>(fcl_tree)));



        shared_ptr<fcl::OcTree<double>> fcl_OcTree_ptr(new fcl::OcTree<double>( shared_ptr<const octomap::OcTree>(&octomap_octree )  ) ) ;
        //fcl::Transform3f fcl_tree_transform3f(collision_object_quaternion_orientation,collision_object_translation);
        //shared_ptr< fcl::CollisionObject<double>   > fcl_tree_collision_object_ptr (new fcl::CollisionObject<double>( shared_ptr<fcl::CollisionGeometry<double>>(fcl_OcTree_ptr), fcl_tree_transform3f) ) ;
        shared_ptr< fcl::CollisionObject<double>   > fcl_tree_collision_object_ptr (new fcl::CollisionObject<double>( shared_ptr<fcl::CollisionGeometry<double>>(fcl_OcTree_ptr)) ) ;



//        fcl::Transform3f fcl_tree_transform3f(collision_object_quaternion_orientation,collision_object_translation);
//        shared_ptr< fcl::CollisionObject   > fcl_tree_collision_object_ptr (new fcl::CollisionObject( fcl_OcTree_ptr, fcl_tree_transform3f) ) ;


        CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
        collision_object_associated_data->setID(link_name);
        fcl_tree_collision_object_ptr->setUserData( collision_object_associated_data );
        vector_of_CollisionObjectAssociatedData_addresses.push_back(collision_object_associated_data);

        broad_phase_collision_manager->registerObject(fcl_tree_collision_object_ptr.get());

        link_name_CollisionObject_entry link_name_CollisionObject;
        link_name_CollisionObject.first=link_name;
        link_name_CollisionObject.second=fcl_tree_collision_object_ptr;
        link_names_CollisionObjects_Container.insert(link_name_CollisionObject );

    }


    void updateCollisionObjectTransform(std::string link_name ,const fcl::Quaterniond collision_object_quaternion_orientation,const fcl::Vector3d collision_object_translation);

    bool IsStateIsValid(int num_max_contacts=1);



    void removeSelfCollisionObject(const std::string &collision_object_name);
    
    void removeWorldCollisionObject(const std::string &collision_object_name);
    
    void removeObject4mCollisionContainer(const std::string &collision_object_name);

    bool checkCollisionAgainstExternalCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager, int num_max_contacts=1);

    shared_ptr<fcl::BroadPhaseCollisionManager<double>>  &getCollisionManager();

    shared_ptr<fcl::BroadPhaseCollisionManager<double>> broad_phase_collision_manager;


    /**
     * @brief Conversion from a PCL pointcloud to octomap::Pointcloud, used internally in OctoMap
     *
     * @param pclCloud
     * @param octomapCloud
     */
    template <class PointT>
    void pointcloudPCLToOctomap(const pcl::PointCloud<PointT>& pclCloud, octomap::Pointcloud& octomapCloud)
    {
      octomapCloud.reserve(pclCloud.points.size());

      typename   pcl::PointCloud<PointT>::const_iterator it;
      for (it = pclCloud.begin(); it != pclCloud.end(); ++it)
      {
        // Check if the point is invalid
        if (!std::isnan (it->x) && !std::isnan (it->y) && !std::isnan (it->z))
          octomapCloud.push_back(it->x, it->y, it->z);
      }
    }


    bool DistanceOfClosestObstacleToRobot(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager,DistanceData &distance_data);

    std::vector <fcl::CollisionObject<double>*> getObjectIncollisionAgainstExternalCollisionManager();

    std::vector < std::pair<fcl::CollisionObject<double>*,fcl::CollisionObject<double>* > > &getSelfCollisionObject();

    std::vector< DistanceInformation> &getSelfDistanceInfo();    

    std::vector< std::pair<std::string, std::string> > getCollisionObjectNames();

    void computeSelfDistanceInfo();

    void printCollisionObject();

};
}// end namespace trajectory_optimization
#endif // COLLISIONDETECTION_HPP
