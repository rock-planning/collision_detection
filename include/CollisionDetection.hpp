#ifndef COLLISIONDETECTOR_HPP_
#define COLLISIONDETECTOR_HPP_

#include <vector>
#include <string>
#include "abstract/AbstractCollisionDetection.hpp"
#include "CollisionConfig.hpp"

#include "fcl_wrapper/FCLCollisionDetection.hpp"


/** \file CollisionDetection.hpp
*    \brief Factory class for the AbstractCollisionDetection class.
*/

namespace collision_detection
{

/**
 * @class CollisionDetector
 * @brief Provides a factory class for the AbstractCollisionDetection class.
 */
class CollisionDetection
{

public:
    /**
    * @brief  constructor
    */
    CollisionDetection();
    /**
    * @brief  destructor
    */
    ~CollisionDetection();

     AbstractCollisionPtr getCollisionDetector(collision_detection::CollisionLibrary library, OctreeDebugConfig octree_debug_config);

};

};

#endif

