#ifndef COLLISIONDATA_HPP
#define COLLISIONDATA_HPP

#include "CollisionObjectAssociatedData.hpp"
#include <fcl/fcl.h>

namespace trajectory_optimization
{


struct CollisionInformation
{
    std::pair<std::string,std::string>  collision_pair;
    std::vector <std::pair<std::string,std::string> > collision_pair_names;
};


struct EnvironmentParameters
{
    std::string input_frame_name;
    double octree_resolution;
    std::string collision_object_name;

};

struct ModelObject
{
    ModelObject():
        operation(trajectory_optimization::RESET),
        model_type(trajectory_optimization::UNDEFINED),
        object_path(""), object_name("") {}

    // this variable says whether the model object should be added or removed
    trajectory_optimization::operation operation;
    // model type: primitives or mesh
    trajectory_optimization::modelTypes model_type;
    // primitive types: box, cylinder, sphere
    trajectory_optimization::primitiveObject primitive_object;
    // mesh file path
    std::string object_path;
    // object name, please give an object name
    std::string object_name;
    // attach the model object to the given link name
    std::string attach_link_name;
    // relative pose of the model object w.r.t to attach_link
    base::Pose relative_pose;
};



struct CollisionData
{
    CollisionData()
    {
        done = false;
    }

    /// @brief Collision request
    fcl::CollisionRequest<double> request;

    /// @brief Collision result
    fcl::CollisionResult<double> result;

    /// @brief Whether the collision iteration can stop
    bool done;
};

struct DistanceData
{
    DistanceData()
    {
        done = false;
    }

    /// @brief Distance request    
    fcl::DistanceRequest<double> request;

    /// @brief Distance result    
    fcl::DistanceResult<double> result;

    /// @brief Whether the distance iteration can stop
    bool done;
};

}// end namespace trajectory_optimization
#endif // COLLISIONDATA_HPP
