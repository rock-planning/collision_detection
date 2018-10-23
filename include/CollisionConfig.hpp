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
    std::vector<double> dimensions;
};

} // end namespace 

#endif
