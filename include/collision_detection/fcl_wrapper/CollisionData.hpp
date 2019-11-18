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

struct CollisionInformation
{
    CollisionInformation(): number_of_collisions(0){}
    /// @brief Number of collisions found.
    size_t number_of_collisions;
    /// @brief vector holding collision object names
    std::vector< std::pair<std::string, std::string> >  collision_object_names;
    /// @breif collision cost
    double collision_cost;
    
    bool stop_after_first_collision;
//     /// @brief vector holding collision object names
//     std::vector< CollisionPair > collision_objects_pair;
};

struct CollisionData
{
    CollisionData(): done(false){}

    /// @brief Collision request
    fcl::CollisionRequest<double> request;
    /// @brief Collision result
    fcl::CollisionResult<double> result;
    /// @brief Information regarding the collision
    CollisionInformation collision_info;
    /// @brief Whether the collision iteration can stop
    bool done;
};




struct DistanceData
{
    DistanceData(): list_of_distance_information(0), done(false){}

    /// @brief Distance request
    fcl::DistanceRequest<double> request;
    /// @brief Distance result    
    fcl::DistanceResult<double> result;
    /// @brief Information regarding the collision
    CollisionInformation collision_info;
    /// @brief Information regarding the collision distance
    std::vector< DistanceInformation> list_of_distance_information;
    /// @brief Whether the distance iteration can stop
    bool done;
};

}// end namespace collision_detection
#endif // COLLISIONDATA_HPP
