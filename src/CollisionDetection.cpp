#include "CollisionDetection.hpp"

namespace collision_detection
{

CollisionDetection::CollisionDetection()
{}

CollisionDetection::~CollisionDetection()
{}


AbstractCollisionPtr CollisionDetection::getCollisionDetector(collision_detection::CollisionLibrary library, OctreeDebugConfig octree_debug_config)
{
	AbstractCollisionPtr collision_detector = NULL;
	

	switch(library)
	{
	
	    case collision_detection::FCL:
	    {
		collision_detector = std::make_shared<FCLCollisionDetection>(octree_debug_config);
		break;
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
