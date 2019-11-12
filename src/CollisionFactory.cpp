#include "collision_detection/CollisionFactory.hpp"

namespace collision_detection
{

CollisionFactory::CollisionFactory()
{}

CollisionFactory::~CollisionFactory()
{}


AbstractCollisionPtr CollisionFactory::getCollisionDetector(collision_detection::CollisionLibrary library, OctreeDebugConfig octree_debug_config, bool use_contact_info)
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
