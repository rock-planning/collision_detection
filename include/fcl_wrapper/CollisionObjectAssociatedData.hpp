#ifndef COLLISIONOBJECTASSOCIATEDDATA_HPP
#define COLLISIONOBJECTASSOCIATEDDATA_HPP

#include <string>
#include <vector>

namespace trajectory_optimization
{

class CollisionObjectAssociatedData
{
private:
    std::string id;
public:
    CollisionObjectAssociatedData();
    std::string& getID();
    void setID(std::string &id);

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
        primitive_type(trajectory_optimization::CYLINDER),
        radius(0.0), height(0.0),
        dimensions(3,0) {}

    primitiveObjectTypes primitive_type;

    // cylinder & sphere
    double radius;
    double height;

    // box
    std::vector<double> dimensions;
};

}// end namespace trajectory_optimization
#endif // COLLISIONOBJECTASSOCIATEDDATA_HPP

