#ifndef COLLISIONDETECTOR_HPP_
#define COLLISIONDETECTOR_HPP_

#include <vector>
#include <string>
#include <abstract/AbstractCollisionDetection.hpp>

#include <fcl_wrapper/FCLCollisionDetection.hpp>


/** \file CollisionDetector.hpp
*    \brief Factory class for the AbstractCollisionDetection class.
*/

namespace trajectory_optimization
{

/**
 * @class CollisionDetector
 * @brief Provides a factory class for the AbstractCollisionDetection class.
 */
class CollisionDetector
{

public:
    /**
    * @brief  constructor
    */
    CollisionDetector();
    /**
    * @brief  destructor
    */
    ~CollisionDetector();

     AbstractCollisionDetection* getCollisionDetector(trajectory_optimization::CollisionLibrary library);

};

};

#endif

