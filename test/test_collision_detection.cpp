#include <base/samples/RigidBodyState.hpp>
#include <collision_detection/CollisionFactory.hpp>

using namespace collision_detection;



collision_detection::AbstractCollisionPtr createCollisionManager(collision_detection::CollisionFactory &collision_factory)
{
    collision_detection::CollisionDetectionConfig coll_config;
    coll_config.collision_library = collision_detection::FCL;
    coll_config.collision_info_type = collision_detection::MULTI_CONTACT; 
    coll_config.calculate_distance_information = true;
    coll_config.stop_after_first_collision = false;
    coll_config.max_num_collision_contacts = 2;

    return collision_factory.getCollisionDetector(coll_config);
}

void createCollisionObjects(collision_detection::AbstractCollisionPtr &collision_detector)
{
    // sphere 1
    double radius_1 = 0.2;
    std::string object_1_name = "sphere_1";
    base::Pose object_1_pose;
    
    // sphere 2
    double radius_2 = 0.2;
    std::string object_2_name = "sphere_2";
    base::Pose object_2_pose;
    
    
    collision_detector->registerSphereToCollisionManager(radius_1, object_1_name, object_1_pose, 1.0);
    collision_detector->registerSphereToCollisionManager(radius_2, object_2_name, object_2_pose, 1.0);
}

void printDistanceInformation(const collision_detection::AbstractCollisionPtr &collision_detector)
{
    std::vector<collision_detection::DistanceInformation> distance_info;
    //cs.robot_model_->getRobotDistanceToCollisionInfo(distance_info);
    distance_info = collision_detector->getCollisionDistanceInformation();

    for(size_t i=0; i<distance_info.size(); i++)
    {
        std::cout<<"Penetration depth between "<<distance_info.at(i).object1<<" and "<<distance_info.at(i).object2<<" is "<<distance_info.at(i).min_distance<<std::endl;
        std::cout<<"contact_normal = \n"<< distance_info.at(i).contact_normal<<std::endl;
    }
}

std::shared_ptr<octomap::OcTree> generateOctomap()
{
    std::shared_ptr<octomap::OcTree> tree (new octomap::OcTree(0.5));

    float logodds = -5.51106;

    octomap::OcTreeKey maxKey = tree->coordToKey(3.5, 3.5, 3.5);
    octomap::OcTreeKey minKey = tree->coordToKey(-3.5, -3.5, -3.5);
 
    octomap::OcTreeKey k;
    for (k[0] = minKey[0]; k[0] < maxKey[0]; ++k[0])
    {
        for (k[1] = minKey[1]; k[1] < maxKey[1]; ++k[1])
        {
            for (k[2] = minKey[2]; k[2] < maxKey[2]; ++k[2])
            {
                tree->updateNode(k, logodds);
            }
        }
    }
    logodds = 10.0;
    octomap::OcTreeKey obs_key = tree->coordToKey(0, 2.0, 0);
    tree->updateNode(obs_key, logodds);
    obs_key = tree->coordToKey(0, 2.20, 0);
    tree->updateNode(obs_key, logodds);
    obs_key = tree->coordToKey(0.0, 2.5, 0);
    tree->updateNode(obs_key, logodds);
    obs_key = tree->coordToKey(0.0, -1.5, 0);
    tree->updateNode(obs_key, logodds);
    obs_key = tree->coordToKey(0.0, -2.20, 0);
    tree->updateNode(obs_key, logodds);
    obs_key = tree->coordToKey(0.0, -2.5, 0);
    tree->updateNode(obs_key, logodds);
    tree->updateInnerOccupancy();
    // if you want one can save the octree
    //tree->writeBinary("./data.bt");
    return tree;
}                            

void createCollisionObjects(collision_detection::AbstractCollisionPtr &robot_collision_detector, 
                            collision_detection::AbstractCollisionPtr &world_collision_detector)
{
    // sphere 1
    double radius_1 = 0.1;
    std::string object_1_name = "sphere1";
    base::Pose object_1_pose;
    object_1_pose.position.setZero();
    object_1_pose.orientation.setIdentity();
    
    // sphere 2
    double radius_2 = 0.1;
    std::string object_2_name = "sphere2";
    base::Pose object_2_pose;
    object_2_pose.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    object_2_pose.orientation.setIdentity();
    
    double collision_padding = 1.0;
    
    // registering the robot collision to its collision manager
    robot_collision_detector->registerSphereToCollisionManager(radius_1, object_1_name, object_1_pose, collision_padding);
    robot_collision_detector->registerSphereToCollisionManager(radius_2, object_2_name, object_2_pose, collision_padding);
    
    // create an octomap as environment representation
    std::shared_ptr<octomap::OcTree> tree = generateOctomap();
    std::string env_object_name = "environment";
    base::Pose env_pose;
    env_pose.position.setZero();
    env_pose.orientation.setIdentity();
    // registering the octree to the world collision manager
    world_collision_detector->registerOctreeToCollisionManager(tree, env_pose, env_object_name );

}
  
void checkForCollision(const collision_detection::AbstractCollisionPtr &robot_collision_detector)
{
    double collision_cost = 0.0;
    if(robot_collision_detector->isCollisionsOccured(collision_cost))
    {
        std::cout<<"Collision Occured between: \n";
        std::vector< std::pair<std::string, std::string> > collision_objects_name = robot_collision_detector->getCollidedObjectsNames();
        for(size_t i = 0; i < collision_objects_name.size(); i++)
            std::cout<<collision_objects_name.at(i).first<<"  "<<collision_objects_name.at(i).second<<std::endl;        
    }
    else
        std::cout<<"There is no collision"<<std::endl;
}

int main()
{
    // create the collision factor and assign fcl based robot and world collision detectors
    collision_detection::CollisionFactory collision_factory;
    collision_detection::AbstractCollisionPtr robot_collision_detector = createCollisionManager(collision_factory);
    collision_detection::AbstractCollisionPtr world_collision_detector = createCollisionManager(collision_factory);
    // assign the world collision detector to the robot collision detector
    robot_collision_detector->assignWorldDetector(world_collision_detector);
    // create two robot object and an environment object
    createCollisionObjects(robot_collision_detector, world_collision_detector);
    std::cout<<"\n";
    // Print collision object
    std::cout<<"---------------- Robot Collision Objects----------------"<<std::endl;
    robot_collision_detector->printCollisionObject();
    std::cout<<"---------------- World Collision Objects----------------"<<std::endl;
    world_collision_detector->printCollisionObject();
    std::cout<<"\n";
    
    std::cout<<"CASE 1: Now we check collision for the initial state"<<std::endl;
    // Now we check collision for the initial state
    checkForCollision(robot_collision_detector);
    printDistanceInformation(robot_collision_detector);
     
    std::cout<<"\nCASE 2: Now we check collision after moving the sphere2"<<std::endl;
    //Now we move the sphere2 towards sphere1
    base::Pose new_pose;
    new_pose.position = Eigen::Vector3d(0.05, 0.0, 0.0);
    new_pose.orientation.setIdentity();
    robot_collision_detector->updateCollisionObjectTransform("sphere2", new_pose);
    // Now we check collision for this state
    checkForCollision(robot_collision_detector);
    printDistanceInformation(robot_collision_detector);
    
    std::cout<<"\nCASE 3: Now we check collision with environment"<<std::endl;
    new_pose.position = Eigen::Vector3d(0.0, -2.5, 0.0);
    robot_collision_detector->updateCollisionObjectTransform("sphere1", new_pose);
    new_pose.position = Eigen::Vector3d(0.0, 2.5, 0.0);
    robot_collision_detector->updateCollisionObjectTransform("sphere2", new_pose);
    // Now we check collision for this state
    checkForCollision(robot_collision_detector);
    printDistanceInformation(robot_collision_detector);
   
    
    return 0;
}

