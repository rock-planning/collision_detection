#include <abstract/AbstractCollisionDetection.hpp>

namespace collision_detection
{

AbstractCollisionDetection::AbstractCollisionDetection()
{}

AbstractCollisionDetection::~AbstractCollisionDetection()
{}

void AbstractCollisionDetection::setDisabledCollisionPairs(std::vector<srdf::Model::DisabledCollision> &disabled_collisions)
{
     disabled_collisions_ = disabled_collisions;
}

void AbstractCollisionDetection::addDisabledCollisionPairs(srdf::Model::DisabledCollision &disabled_collision)
{
     disabled_collisions_.push_back(disabled_collision);
}



void AbstractCollisionDetection::removeDisabledCollisionLink(const std::string & link)
{
    remove_link_ = link;
    disabled_collisions_.erase(std::remove_if(disabled_collisions_.begin(), disabled_collisions_.end(), boost::bind(&AbstractCollisionDetection::isLinkListed, this, _1)), disabled_collisions_.end());

}

bool AbstractCollisionDetection::isLinkListed(srdf::Model::DisabledCollision const &remove_link)
{
    return ((remove_link.link1_ == remove_link_) || (remove_link.link2_ == remove_link_) );
}

static bool linksToBeChecked( std::string first_link_name, std::string second_object_name )
{
    if(first_link_name == second_object_name  )
    {
        return false;
    }

    for(std::size_t i = 0; i < AbstractCollisionDetection::disabled_collisions_.size(); i++ )
    {
        if( (first_link_name    == AbstractCollisionDetection::disabled_collisions_.at(i).link1_ &&  second_object_name  == AbstractCollisionDetection::disabled_collisions_.at(i).link2_) || 
            (second_object_name == AbstractCollisionDetection::disabled_collisions_.at(i).link1_ &&  first_link_name     == AbstractCollisionDetection::disabled_collisions_.at(i).link2_)  )
        {
            return false;
        }
    }
    return true;
}




}
