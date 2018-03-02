#include <fcl_wrapper/CollisionObjectAssociatedData.hpp>

namespace trajectory_optimization
{

CollisionObjectAssociatedData::CollisionObjectAssociatedData()
{
}


std::string& CollisionObjectAssociatedData::getID()
{
    return this->id;
}

void CollisionObjectAssociatedData::setID(std::string &id)
{
    this->id=id;
}

}// end namespace trajectory_optimization
