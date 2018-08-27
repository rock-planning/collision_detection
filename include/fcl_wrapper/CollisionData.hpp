#ifndef COLLISIONDATA_HPP
#define COLLISIONDATA_HPP

#include "CollisionObjectAssociatedData.hpp"
#include <fcl/fcl.h>

namespace collision_detection
{


struct CollisionInformation
{
    std::pair<std::string,std::string>  collision_pair;
    std::vector <std::pair<std::string,std::string> > collision_pair_names;
};



struct ModelObject
{
    ModelObject():
        operation(collision_detection::RESET),
        model_type(collision_detection::UNDEFINED),
        object_path(""), object_name("") {}

    // this variable says whether the model object should be added or removed
    collision_detection::operation operation;
    // model type: primitives or mesh
    collision_detection::modelTypes model_type;
    // primitive types: box, cylinder, sphere
    collision_detection::primitiveObject primitive_object;
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

}// end namespace collision_detection
#endif // COLLISIONDATA_HPP
