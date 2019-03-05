#ifndef _COLLISIONCONFIG_HPP_
#define _COLLISIONCONFIG_HPP_

#include <string>
#include <vector>

namespace collision_detection
{
   
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
    dimensions(3,0) {}

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

} // end namespace 

#endif
