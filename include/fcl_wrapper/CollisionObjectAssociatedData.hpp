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

}// end namespace collision_detection
#endif // COLLISIONOBJECTASSOCIATEDDATA_HPP

