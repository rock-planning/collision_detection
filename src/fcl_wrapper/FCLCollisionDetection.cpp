#include <fcl_wrapper/FCLCollisionDetection.hpp>
#include <algorithm>


namespace collision_detection
{

/////////////////////////////////////////////////////out of class variables for defaultCollisionFunction //////////////////////////////////////////////////////////////////
int number_of_time_defaultCollisionFunction_has_been_called=0;
int number_of_time_defaultDistanceFunction_has_been_called=0;
int number_of_links_without_collision=0;
int number_of_times_collision_has_been_ignored=0;


/*number_of_collision_between_links counts number of collision that cause problem and cant't be ignored.
this variable should be set to zero everytime isStateIsValid is called, scince it only should check the current configuration has no collision

*/
int number_of_collision_between_links=0;

// This variable contains collision object names
std::vector< std::pair<std::string, std::string> >  collision_object_names;

///////////////////////////////////////////////out of class variavbles for checking collision against external collision manager//////////////////////////////////////////////

int number_of_time_defaultCollisionFunctionCheckingAgainstexternalCollisionManager_has_been_called=0;
int number_of_collision_between_link_and_externalCollisionManager=0;

std::vector<fcl::CollisionObject<double> *> list_of_collision_objects;

typedef std::pair<fcl::CollisionObject<double> *,fcl::CollisionObject<double> *> CollisionPair;

std::vector< CollisionPair > list_of_self_collision_objects;
std::vector< CollisionPair > list_of_external_collision_objects;
std::vector< DistanceInformation> list_of_self_distance_information;



/////////////////////////////////////////////////out of class  function for defaultCollisionFunction///////////////////////////////////////////////////////


std::string remove_collision_object_4m_collisionManager;



bool defaultCollisionFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_)
{

//    #define DEFAULTCOLLISIONFUNCTION_LOG
//    #define DEFAULTCOLLISIONFUNCTION_LOG_CHECKING_COLLISION
//    #define DEFAULTCOLLISIONFUNCTION_LOG_IGNORING_COLLISION
//    #define DEFAULTCOLLISIONFUNCTION_LOG_SHOWING_COLLISION
//    #define DEFAULTCOLLISIONFUNCTION_LOG_NO_COLLISION

    #ifdef DEFAULTCOLLISIONFUNCTION_LOG
    std::cout<<""<<std::endl;
    std::cout<<"----------------------------------- DEFAULTCOLLISIONFUCNTION  -------------------------------------" <<std::endl;
    #endif

    number_of_time_defaultCollisionFunction_has_been_called++;

    CollisionData* cdata = static_cast<CollisionData*>(cdata_);
    const fcl::CollisionRequest<double>& request = cdata->request;
    fcl::CollisionResult<double>& result = cdata->result;
    if(cdata->done)
    {
        return true;
    }

    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o1->getUserData());
    o2_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o2->getUserData());

    std::string first_object_name= o1_collision_object_associated_data->getID();
    std::string second_object_name=  o2_collision_object_associated_data->getID();

    first_object_name=first_object_name.substr(0,first_object_name.find_last_of("_"))  ;
    second_object_name=second_object_name.substr(0,second_object_name.find_last_of("_"));

    #ifdef DEFAULTCOLLISIONFUNCTION_LOG_CHECKING_COLLISION
    std::cout<< "checking collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
    #endif

    if(AbstractCollisionDetection::linksToBeChecked(first_object_name, second_object_name ))
    {
        fcl::collide(o1, o2, request, result);
        if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
        {
            cdata->done = true;
        }
        if( result.isCollision() )
        {
            CollisionPair collision_pair;
            collision_pair.first=o1;
            collision_pair.second=o2;
            list_of_self_collision_objects.push_back(collision_pair);
//            list_of_self_collision_objects.push_back(o1);
//            list_of_self_collision_objects.push_back(o2);

            #ifdef DEFAULTCOLLISIONFUNCTION_LOG_SHOWING_COLLISION
            std::cout<< "there is collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
            #endif

            std::pair<std::string, std::string> name_pair;

            name_pair.first  = first_object_name;
            name_pair.second = second_object_name;

            collision_object_names.push_back(name_pair);

            number_of_collision_between_links++;
        }
        else
        {
            #ifdef DEFAULTCOLLISIONFUNCTION_LOG_NO_COLLISION
//            std::cout<< "there is NO collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
            #endif
            number_of_links_without_collision++;
        }
    }
    else
    {
        #ifdef DEFAULTCOLLISIONFUNCTION_LOG_IGNORING_COLLISION
        std::cout<<"the collision between "<<first_object_name<<" and " <<second_object_name<<" has been IGNORED." <<std::endl;
        #endif
        number_of_times_collision_has_been_ignored++;

    }
    #ifdef DEFAULTCOLLISIONFUNCTION_LOG
    std::cout<<"------------------------------------------------------------------------------------------------------------------" <<std::endl;
    #endif

    return cdata->done;


}


bool defaultCollisionFunctionCheckingAgainstexternalCollisionManager(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_)
{

//#define DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
#ifdef DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
std::cout<< ""<<std::endl;
std::cout<<"-----------------------DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG-------------------------------------" <<std::endl;
#endif
    number_of_time_defaultCollisionFunctionCheckingAgainstexternalCollisionManager_has_been_called++;

    CollisionData* cdata = static_cast<CollisionData*>(cdata_);
    const fcl::CollisionRequest<double>& request = cdata->request;
    fcl::CollisionResult<double>& result = cdata->result;
    if(cdata->done)
    {
        return true;
    }


    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o1->getUserData());
    o2_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o2->getUserData());

    std::string first_object_name= o1_collision_object_associated_data->getID();
    std::string second_object_name=  o2_collision_object_associated_data->getID();

    first_object_name=first_object_name.substr(0,first_object_name.find_last_of("_"))  ;
    second_object_name=second_object_name.substr(0,second_object_name.find_last_of("_"));


    #ifdef DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
    std::cout<< "checking collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
    #endif

    fcl::collide(o1, o2, request, result);
    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    {
        cdata->done = true;
    }
    if( result.isCollision() )
    {
        list_of_collision_objects.push_back(o1);
        list_of_collision_objects.push_back(o2);

        CollisionPair collision_pair;
        collision_pair.first=o1;
        collision_pair.second=o2;
        list_of_external_collision_objects.push_back(collision_pair);

        //#ifdef DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
        std::cout<< "there is collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
        //#endif

        std::pair<std::string, std::string> name_pair;

        name_pair.first  = first_object_name;
        name_pair.second = second_object_name;

        collision_object_names.push_back(name_pair);

        number_of_collision_between_link_and_externalCollisionManager++;
    }
    else
    {
        #ifdef DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
        //std::cout<< "there is NO collision between " <<first_object_name <<" and " <<second_object_name <<std::endl;
        #endif
    }

    #ifdef DEFAULTCOLLISIONFUNCTIONCHECKINGAGAINSTEXTERNALCOLLISIONMANAGER_LOG
    std::cout<<"------------------------------------------------------------------------------------------------------------------" <<std::endl;
    #endif

    return cdata->done;
}



bool isObjectListedInCollisionObjectAssociatedData(CollisionObjectAssociatedData *remove_object)
{
    return ( remove_object->getID() == remove_collision_object_4m_collisionManager);
}

///////////////////////////////////////////////////////////////End of out of class members and variavbles////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////out of class  function for distance function////////////////////////////////////////////////////////////////////
bool defaultDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double& dist)
{    
//    #define DEFAULTDISTANCEFUNCTION_LOG
    number_of_time_defaultDistanceFunction_has_been_called++;
    DistanceData* cdata = static_cast<DistanceData*>(cdata_);
    const fcl::DistanceRequest<double>& request = cdata->request;
    fcl::DistanceResult<double>& result = cdata->result;

    if(cdata->done)
    {
        dist = result.min_distance;
        return true;
    }
    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o1->getUserData());
    o2_collision_object_associated_data=static_cast<CollisionObjectAssociatedData*>(o2->getUserData());
    std::string fist_object_name= o1_collision_object_associated_data->getID();
    std::string second_object_name=  o2_collision_object_associated_data->getID();
    fist_object_name=fist_object_name.substr(0,fist_object_name.find_last_of("_"))  ;
    second_object_name=second_object_name.substr(0,second_object_name.find_last_of("_"));


//    std::cout<<"checking distance between  "<<fist_object_name<<" and " <<second_object_name<<std::endl;

    if(AbstractCollisionDetection::linksToBeChecked(fist_object_name, second_object_name ))
    {
        fcl::distance(o1, o2, request, result);
        dist = result.min_distance;

        if(dist > 0)
        {
            #ifdef DEFAULTDISTANCEFUNCTION_LOG
            std::cout<<"the distance is between  "<<fist_object_name<<" and " <<second_object_name<<" and the dist is "<<result.min_distance <<std::endl;
            std::cout<<"------------------------------------------------------------------------------------------------------------" <<std::endl;
            #endif

            DistanceInformation distance_information;
            distance_information.object1=fist_object_name;
            distance_information.object2=second_object_name;
            distance_information.distance=result.min_distance;
            list_of_self_distance_information.push_back(distance_information);

        }else
        {
            #ifdef DEFAULTDISTANCEFUNCTION_LOG
            std::cout<<fist_object_name<<" and " <<second_object_name<<" are in collision "<<std::endl;
            #endif
        }

    }
    return cdata->done;
}


///////////////////////////////////////////////////////////////End of out of class members and variavbles////////////////////////////////////////////////////////////

FCLCollisionDetection::FCLCollisionDetection()
{
    broad_phase_collision_manager.reset(new fcl::DynamicAABBTreeCollisionManager<double> );
}

FCLCollisionDetection::~FCLCollisionDetection()
{

    for(std::size_t i=0;i<vector_of_CollisionObjectAssociatedData_addresses.size();i++)
    {
        delete vector_of_CollisionObjectAssociatedData_addresses.at(i);
    }

    //std::cout<<" link_names_CollisionObjects_Container.size(): "  << link_names_CollisionObjects_Container.size()  <<std::endl;
}


void FCLCollisionDetection::registerMeshToCollisionManager(const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, const std::string &link_name, 
					const base::Pose &collision_object_pose, const double &link_padding)
{
//        #define registerMeshToCollisionObjectManager_LOG
    #ifdef registerMeshToCollisionObjectManager_LOG
         std::cout<<"abs_path_to_mesh_file:" <<abs_path_to_mesh_file <<std::endl;
    #endif

    std::vector<fcl::Triangle> triangles;
    std::vector<fcl::Vector3d> vertices;

    scale_mesh_(0) = mesh_scale(0) * link_padding;
    scale_mesh_(1) = mesh_scale(1) * link_padding;
    scale_mesh_(2) = mesh_scale(2) * link_padding;

    mesh_loader.extractTrianglesAndVerticesFromMesh(abs_path_to_mesh_file , triangles, vertices , scale_mesh_(0), scale_mesh_(1), scale_mesh_(2));
    
    
    registerMeshToCollisionManager(link_name, collision_object_pose, triangles, vertices);
    

    return;
}
/*
link_name is collision_object_name, in fact we add  number to the end of link name to make a name for collision object name, 
wrist_1, wrist_2, wrist_3, wrist_4,....
 
*/
void FCLCollisionDetection::registerMeshToCollisionManager(const std::string &link_name, const base::Pose &collision_object_pose, 
							   const std::vector<fcl::Triangle> &triangles, const std::vector<fcl::Vector3d> &vertices)
{
//  #define registerMeshToCollisionObjectManager_LOG
    #ifdef registerMeshToCollisionObjectManager_LOG
    #endif

    
    shared_ptr<fcl::BVHModel<fcl::OBBRSS<double> > >  fcl_mesh_ptr(new fcl::BVHModel<fcl::OBBRSS<double>>);
    fcl_mesh_ptr->beginModel();
    fcl_mesh_ptr->addSubModel(vertices,triangles);
    fcl_mesh_ptr->endModel();
    //fcl::Transform3d mesh_transform3f(collision_object_quaternion_orientation,collision_object_translation);
    //shared_ptr<fcl::CollisionObject<double>>   mesh_collision_object ( new fcl::CollisionObject<double>( fcl_mesh_ptr , mesh_transform3f )  );
    shared_ptr<fcl::CollisionObject<double>> mesh_collision_object_ptr ( new fcl::CollisionObject<double>( fcl_mesh_ptr ) );
    
    registerCollisionObjectToCollisionManager(link_name, collision_object_pose, mesh_collision_object_ptr);  

    return;
}

void FCLCollisionDetection::registerBoxToCollisionManager(const double &box_x, const double &box_y, const double &box_z, const std::string &link_name ,
							  const base::Pose &collision_object_pose, const double &link_padding )
    
{

    //std::cout<<"registerBoxToCollisionObjectManager, link_name is: " <<link_name <<std::endl;
    shared_ptr<fcl::Box<double> > fcl_box_ptr(new fcl::Box<double>(box_x*link_padding, box_y*link_padding, box_z*link_padding));
    
    shared_ptr< fcl::CollisionObject<double>   > box_collision_object_ptr (new fcl::CollisionObject<double>( fcl_box_ptr ) ) ;
    
    registerCollisionObjectToCollisionManager(link_name, collision_object_pose, box_collision_object_ptr);  

}
void FCLCollisionDetection::registerCylinderToCollisionManager(	const double &radius, const double &length, const std::string &link_name ,
								const base::Pose &collision_object_pose, const double &link_padding )
{
    shared_ptr<fcl::Cylinder<double> > fcl_cylinder_ptr(new fcl::Cylinder<double>(radius*link_padding,length*link_padding));
    
    shared_ptr< fcl::CollisionObject<double>   > cylinder_collision_object_ptr (new fcl::CollisionObject<double>( fcl_cylinder_ptr ) ) ;
    
    registerCollisionObjectToCollisionManager(link_name, collision_object_pose, cylinder_collision_object_ptr);  

}
void FCLCollisionDetection::registerSphereToCollisionManager(const double &radius, const std::string &link_name , const base::Pose &collision_object_pose, const double &link_padding )
{
    shared_ptr<fcl::Sphere<double> > fcl_sphere_ptr(new fcl::Sphere<double> (radius*link_padding));   
    
    shared_ptr< fcl::CollisionObject<double>   > sphere_collision_object_ptr (new fcl::CollisionObject<double>( fcl_sphere_ptr  ) ) ;
    
    registerCollisionObjectToCollisionManager(link_name, collision_object_pose, sphere_collision_object_ptr);  

}

void FCLCollisionDetection::registerCollisionObjectToCollisionManager(const std::string &link_name, const base::Pose &collision_object_pose, shared_ptr< fcl::CollisionObject<double> > &collision_object )
{
    collision_object->setTransform(collision_object_pose.orientation, collision_object_pose.position);
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    collision_object->setUserData( collision_object_associated_data );
    vector_of_CollisionObjectAssociatedData_addresses.push_back(collision_object_associated_data);


    broad_phase_collision_manager->registerObject(collision_object.get());
    link_name_CollisionObject_entry link_name_CollisionObject;
    link_name_CollisionObject.first  = link_name;
    link_name_CollisionObject.second = collision_object;
    link_names_CollisionObjects_Container.insert(link_name_CollisionObject );
}
/*
void FCLCollisionDetection::registerOctomapOctree()
{
    fcl::OcTree* fcl_tree = new fcl::OcTree(shared_ptr<const octomap::OcTree>(generateOcTree()));
    fcl::CollisionObject<double> tree_obj((shared_ptr<fcl::CollisionGeometry>(fcl_tree)));

}

void FCLCollisionDetection::registerFclOctree()
{
    fcl::OcTree* fcl_tree = new fcl::OcTree(shared_ptr<const octomap::OcTree>(generateOcTree()));
    fcl::CollisionObject<double> tree_obj((shared_ptr<fcl::CollisionGeometry>(fcl_tree)));

}


void FCLCollisionDetection::registerOctamapPointCloud(octomap::point3d origin,octomap::Pointcloud cloud)
{

}

template <class PointT>
void FCLCollisionDetection::registerPCLPointCloud(const pcl::PointCloud<PointT>& pclCloud, Pointcloud& octomap::octomapCloud)
{

}
*/


void FCLCollisionDetection::removeSelfCollisionObject(const std::string &collision_object_name)
{
    //find the collision object -
    link_names_CollisionObjects::iterator it=link_names_CollisionObjects_Container.find(collision_object_name+"_0");

    //unregister the collision object from collision manager.
    broad_phase_collision_manager->unregisterObject(it->second.get());

    //remove collision object from collision containter
    removeObject4mCollisionContainer(collision_object_name);
}

void FCLCollisionDetection::removeWorldCollisionObject(const std::string &collision_object_name)
{
    //find the collision object -
    link_names_CollisionObjects::iterator it = link_names_CollisionObjects_Container.find(collision_object_name);

    if (it == link_names_CollisionObjects_Container.end())
    {
        std::cout<<"There is no object with a name "<<collision_object_name<<" in the world. No object is deleted"<<std::endl;
        return;
    }
 
    //unregister the collision object from collision manager.
    broad_phase_collision_manager->unregisterObject(it->second.get());

    //remove collision object from collision containter
    removeObject4mCollisionContainer(collision_object_name);
}

void FCLCollisionDetection::removeObject4mCollisionContainer(const std::string &collision_object_name)
{
    remove_collision_object_4m_collisionManager = collision_object_name;

    vector_of_CollisionObjectAssociatedData_addresses.erase(std::remove_if(vector_of_CollisionObjectAssociatedData_addresses.begin(),
                                                                           vector_of_CollisionObjectAssociatedData_addresses.end(),
                                                                           isObjectListedInCollisionObjectAssociatedData),
                                                            vector_of_CollisionObjectAssociatedData_addresses.end());

    //remove the collision object from collisionObjects container
    link_names_CollisionObjects_Container.erase(collision_object_name);
}


void FCLCollisionDetection::updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose)
{
    link_names_CollisionObjects::iterator it=link_names_CollisionObjects_Container.find(link_name);
    //fcl::Transform3d new_transform3f(collision_object_quaternion_orientation,collision_object_translation);
    //it->second->setTransform(new_transform3f);
    it->second->setTransform(collision_object_pose.orientation, collision_object_pose.position);
    it->second->computeAABB();
    broad_phase_collision_manager->update(it->second.get());
}

bool FCLCollisionDetection::checkSelfCollision(int num_max_contacts)
{

//#define StateIsInCollision_LOG

    #ifdef StateIsInCollision_LOG
    std::cout<<"--------------- FCLCollisionDetection: StateIsInCollision ----------"<<std::endl;
    #endif
    CollisionData collision_data;

    collision_data.request.num_max_contacts=num_max_contacts;
    if(num_max_contacts>1)
    {
        collision_data.request.enable_contact=true;
    }
    else
    {
        collision_data.request.enable_contact=false;
    }
    number_of_collision_between_links=0;
    list_of_self_collision_objects.clear();
    collision_object_names.clear();
    broad_phase_collision_manager->collide(&collision_data, defaultCollisionFunction);
    std::vector<fcl::Contact<double>> self_collision_contacts;
    collision_data.result.getContacts(self_collision_contacts);
    this->self_collision_contacts=self_collision_contacts;
    #ifdef StateIsInCollision_LOG
    std::cout<<"self_collision_contacts.size(): " <<self_collision_contacts.size()<<std::endl;

    for(int i=0;i<self_collision_contacts.size();i++)
    {
        std::cout<<"penetration_depth: "<<self_collision_contacts.at(i).penetration_depth<<std::endl;
//        std::cout<<self_collision_contacts.at(i).o1->aabb_center.data[0]   <<std::endl;
//        std::cout<<self_collision_contacts.at(i).o1->aabb_center.data[1]   <<std::endl;
//        std::cout<<self_collision_contacts.at(i).o1->aabb_center.data[2]   <<std::endl;

    }
    std::cout<<"--------  End of IsStateIsValid in FCLCollisionDetection -------"<<std::endl;
    #endif

    if (number_of_collision_between_links==0)
    {
        return true;
    }
    else
    {
        return false;
    }

}


bool FCLCollisionDetection::checkWorldCollision(int num_max_contacts)
{
    checkCollisionAgainstExternalCollisionManager(   static_cast<FCLCollisionDetection*>(world_collision_detector_)->getCollisionManager());
}

bool FCLCollisionDetection::checkCollisionAgainstExternalCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager, int num_max_contacts)
{
//    #define CHECKCOLLISIONAGAINSTEXTERNALCOLLISIONMANAGER_LOG

    #ifdef CHECKCOLLISIONAGAINSTEXTERNALCOLLISIONMANAGER_LOG
         std::cout<<"------------------ FCLCollisionDetection: CHECKCOLLISIONAGAINSTEXTERNALCOLLISION ------------- "<<std::endl;
    #endif
    CollisionData collision_data;
    collision_data.request.num_max_contacts=num_max_contacts;
    if(num_max_contacts>1)
    {
        collision_data.request.enable_contact=true;
    }

    number_of_collision_between_link_and_externalCollisionManager=0;
    list_of_collision_objects.clear();
    list_of_external_collision_objects.clear();
    collision_object_names.clear();
    this->broad_phase_collision_manager->collide( external_broad_phase_collision_manager.get() ,&collision_data, defaultCollisionFunctionCheckingAgainstexternalCollisionManager);

    std::vector<fcl::Contact<double>> collision_contacts;
    collision_data.result.getContacts(collision_contacts);
    this->collision_contacts_against_external_collision_manager =collision_contacts;


    #ifdef CHECKCOLLISIONAGAINSTEXTERNALCOLLISIONMANAGER_LOG
    std::cout<<"env collision contacts: " <<this->collision_contacts_against_external_collision_manager.size()<<std::endl;
    for(int i=0;i<this->collision_contacts_against_external_collision_manager.size();i++)
    {
        std::cout<<"penetration_depth: "<<this->collision_contacts_against_external_collision_manager.at(i).penetration_depth<<std::endl;
    }
    #endif



    /**/
    if (number_of_collision_between_link_and_externalCollisionManager==0)
    {
        return true;
    }
    else
    {
        return false;
    }

//    return true;

}

/**/
bool FCLCollisionDetection::DistanceOfClosestObstacleToRobot(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager,DistanceData &distance_data)
{
    this->broad_phase_collision_manager->distance( external_broad_phase_collision_manager.get(), &distance_data, defaultDistanceFunction);
    if(distance_data.result.min_distance==0)
    {
        return false;// collison or touch
    }
    else
    {
        return true; // everything is fine
    }
}

void FCLCollisionDetection::getCollisionManager(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &collision_manager)
{
    collision_manager=this->broad_phase_collision_manager;
}


shared_ptr<fcl::BroadPhaseCollisionManager<double>> & FCLCollisionDetection::getCollisionManager()
{
    return this->broad_phase_collision_manager;
}


int FCLCollisionDetection::numberOfObjectsInCollisionManger()
{
    std::vector<fcl::CollisionObject<double>*> objs;
    broad_phase_collision_manager->getObjects(objs);
    return objs.size();
}

std::vector <fcl::CollisionObject<double>*> FCLCollisionDetection::getObjectIncollisionAgainstExternalCollisionManager()
{
    return list_of_collision_objects;
}

std::vector<fcl::Contact<double>> &FCLCollisionDetection::getContactsAgainstExternalCollisionManager()
{
    return this->collision_contacts_against_external_collision_manager;
}

std::vector<fcl::Contact<double>> &FCLCollisionDetection::getSelfContacts()
{
    return this->self_collision_contacts;
}

std::vector < std::pair<fcl::CollisionObject<double>*,fcl::CollisionObject<double>* > > &FCLCollisionDetection::getSelfCollisionObject()
{
    return list_of_self_collision_objects;
}

void FCLCollisionDetection::computeSelfDistanceInfo()
{
    DistanceData distance_data;
    list_of_self_distance_information.clear();
    this->broad_phase_collision_manager->distance(&distance_data, defaultDistanceFunction);
}

std::vector< DistanceInformation> &FCLCollisionDetection::getSelfDistanceInfo()
{
    return list_of_self_distance_information;
}

void FCLCollisionDetection::printCollisionObject()
{
    std::cout<<"The collision object containter contains "<<link_names_CollisionObjects_Container.size()<<" objects with the following names "<<std::endl;

    for(link_names_CollisionObjects::iterator it = link_names_CollisionObjects_Container.begin(); it!=link_names_CollisionObjects_Container.end();it++)
    {
        std::cout<<it->first<<std::endl;
    }
    
    std::cout<<"---- End of print collision object funtion ------ "<<std::endl;
}

std::vector< std::pair<std::string, std::string> > FCLCollisionDetection::getCollisionObjectNames()
{
   return collision_object_names;
}

}// end namespace collision_detection


