#ifndef COLLISIONDATA_HPP
#define COLLISIONDATA_HPP

#include "collision_detection/fcl_wrapper/CollisionObjectAssociatedData.hpp"
#include <fcl/fcl.h>

namespace collision_detection
{


// struct CollisionInformation
// {
//     std::pair<std::string,std::string>  collision_pair;
//     std::vector <std::pair<std::string,std::string> > collision_pair_names;
// };



typedef std::pair<fcl::CollisionObject<double> *,fcl::CollisionObject<double> *> CollisionPair;

struct CollisionData
{
    CollisionData(): done(false){}

    /// @brief Collision request
    fcl::CollisionRequest<double> request;
    /// @brief Collision result
    fcl::CollisionResult<double> result;
    /// @brief Number of collisions found
    size_t number_of_collisions;
    /// @brief vector holding collision object names
    std::vector< std::pair<std::string, std::string> >  collision_object_names;
    /// @brief vector holding collision object names
    std::vector< CollisionPair > collision_objects_pair;

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
