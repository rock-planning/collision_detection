#ifndef _COLLISIONCONFIG_HPP_
#define _COLLISIONCONFIG_HPP_

#include <string>
#include <vector>
#include <Eigen/Core>

namespace collision_detection
{

enum CollisionLibrary
{
    FCL
};

enum Operation
{
    ADD,
    REMOVE,
    RESET
};

enum ModelTypes
{
    PRIMITIVES,
    MESH,
    OCTREE,
    UNDEFINED
};

enum PrimitiveObjectTypes
{
    BOX,
    CYLINDER,
    SPHERE
};

struct PrimitiveObject
{
    PrimitiveObject():
    primitive_type(collision_detection::CYLINDER),
    radius(0.0), height(0.0),
    dimensions(3) {}

    PrimitiveObjectTypes primitive_type;

    // cylinder & sphere
    double radius;
    double height;

    // box
    Eigen::Vector3d dimensions;
};

struct OctreeDebugConfig
{
    OctreeDebugConfig():
    save_octree(false), save_octree_path("./"),
    save_octree_filename("octree.bt") {}

    bool save_octree;
    std::string save_octree_path;
    std::string save_octree_filename;
};

/** Collision Information. 
 *  This enum is used to decide whether to calculate only the distance information or multi contact collision information from the collision library.
 */
enum CollisionInfoType
{
    DISTANCE,           /**< Distance info usng GTK */
    MULTI_CONTACT       /**< Multi contact info */
};

struct CollisionLinkName
{
    CollisionLinkName(const std::string &link_1="", const std::string &link_2=""):
    link_1(link_1), link_2(link_2){}
    
    std::string link_1;
    std::string link_2;
};

struct CollisionLinksName
{
    std::vector<CollisionLinkName> collision_link_names;
};

struct CollisionDetectionConfig
{
    CollisionDetectionConfig(): collision_library(FCL), collision_info_type(DISTANCE), stop_after_first_collision(true), calculate_distance_information(false),
                                max_num_collision_contacts(1) {}
    CollisionLibrary collision_library;
    CollisionInfoType collision_info_type;
    /// @brief maximum number of collision contact.
    /// Usage: For sampling based planner, it is enough to invalid a state space if there is a collision in just one link.
    ///        On the other hand if you use optimzation based planners, it is better to know all the collisions. 
    ///        As this collision information could be used to calculate the collision cost for that iteration.
    bool stop_after_first_collision;
    /// @brief calculate the distance informations for all the collision links.
    /// It is expensive to calculate, so by default it is set to false
    bool calculate_distance_information;
    /// @breif number of collision contacts. By default it is set to one. Please dont set any other value, unless you want that information.
    std::size_t max_num_collision_contacts; 
    OctreeDebugConfig env_debug_config;
};

} // end namespace 

#endif
