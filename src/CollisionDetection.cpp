#include "CollisionDetection.hpp"

namespace trajectory_optimization
{

CollisionDetector::CollisionDetector()
{}

CollisionDetector::~CollisionDetector()
{}


AbstractCollisionDetection* CollisionDetector::getCollisionDetector(trajectory_optimization::CollisionLibrary library)
{
	AbstractCollisionDetection *collision_detector = NULL;

	switch(library)
	{
		case FCL:
		{}
		default:
		{
			std::cout<<"No collision library selected"<<std::endl;
			return NULL;
		}
	}
	return collision_detector;
}


}
