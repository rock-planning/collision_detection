#include "CollisionDetection.hpp"

namespace collision_detection
{

CollisionDetection::CollisionDetection()
{}

CollisionDetection::~CollisionDetection()
{}


AbstractCollisionPtr CollisionDetection::getCollisionDetector(collision_detection::CollisionLibrary library)
{
	AbstractCollisionPtr collision_detector = NULL;

	switch(library)
	{
		case FCL:
		{
		  collision_detector = std::shared_ptr< FCLCollisionDetection>();
		}
		default:
		{
			std::cout<<"No collision library selected"<<std::endl;
			return NULL;
		}
	}
	return collision_detector;
}


}
