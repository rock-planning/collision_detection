#ifndef _COLLISIONCONFIG_HPP_
#define _COLLISIONCONFIG_HPP_

#include <string>
#include <vector>

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
    base::Vector3d dimensions;
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
enum CollisionInfo
{
    DISTANCE,           /**< Distance info usng GTK */
    MULTI_CONTACT       /**< Multi contact info */
};

struct CollisionDetectionConfig
{
    CollisionDetectionConfig():collision_library(FCL), collision_info(DISTANCE), max_num_collision(1){}
    CollisionLibrary collision_library;
    CollisionInfo collision_info;
    bool max_num_collision;
    OctreeDebugConfig env_debug_config;
};

} // end namespace 

#endif
