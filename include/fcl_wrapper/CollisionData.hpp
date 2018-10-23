#ifndef COLLISIONDATA_HPP
#define COLLISIONDATA_HPP

#include "CollisionObjectAssociatedData.hpp"
#include <fcl/fcl.h>

namespace collision_detection
{


// struct CollisionInformation
// {
//     std::pair<std::string,std::string>  collision_pair;
//     std::vector <std::pair<std::string,std::string> > collision_pair_names;
// };


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
