#ifndef COLLISIONOBJECTASSOCIATEDDATA_HPP
#define COLLISIONOBJECTASSOCIATEDDATA_HPP

#include <string>
#include <vector>

namespace collision_detection
{

class CollisionObjectAssociatedData
{
private:
    std::string id;
public:
    CollisionObjectAssociatedData();
    std::string& getID();
    void setID(const std::string &id);

};

enum operation
{
    ADD,
    REMOVE,
    RESET
};

enum modelTypes
{
    PRIMITIVES,
    MESH,
    UNDEFINED
};

enum primitiveObjectTypes
{
    BOX,
    CYLINDER,
    SPHERE
};

struct primitiveObject
{
    primitiveObject():
        primitive_type(collision_detection::CYLINDER),
        radius(0.0), height(0.0),
        dimensions(3,0) {}

    primitiveObjectTypes primitive_type;

    // cylinder & sphere
    double radius;
    double height;

    // box
    std::vector<double> dimensions;
};

}// end namespace collision_detection
#endif // COLLISIONOBJECTASSOCIATEDDATA_HPP

