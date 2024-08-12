#include "collision_detection/fcl_wrapper/FCLCollisionDetection.hpp"
#include <algorithm>

namespace collision_detection
{
////////////////////// Out of class variables and functions for collision //////////////////////////////////////
// std::vector< DistanceInformation> list_of_distance_information;
std::string remove_collision_object_4m_collisionManager;

bool isObjectListedInCollisionObjectAssociatedData(CollisionObjectAssociatedData *remove_object)
{
    return ( remove_object->getID() == remove_collision_object_4m_collisionManager);
}

bool defaultCollisionFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_)
{
    CollisionData* cdata                            = static_cast<CollisionData*>(cdata_);
    const fcl::CollisionRequest<double>& request    = cdata->request;
    fcl::CollisionResult<double>& result            = cdata->result;

    //why we do this ?
    if(cdata->done)
    {
        // ??
        return true;
    }
    
    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o1->getUserData());
    o2_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o2->getUserData());

    std::string first_object_name  = o1_collision_object_associated_data->getID();
    std::string second_object_name = o2_collision_object_associated_data->getID();

    first_object_name  = first_object_name.substr(0,first_object_name.find_last_of("_"))  ;
    second_object_name = second_object_name.substr(0,second_object_name.find_last_of("_"));

    LOG_DEBUG_S<<"[defaultCollisionFunction]: Checking collision between "<<first_object_name.c_str()<<" and "<<second_object_name.c_str();

    if(AbstractCollisionDetection::linksToBeChecked(first_object_name, second_object_name ))
    {
        fcl::collide(o1, o2, request, result);

        if( result.isCollision() )
        {
            LOG_DEBUG_S<<"[defaultCollisionFunction]: There is collision between "<<first_object_name.c_str()<<" and "<<second_object_name.c_str();

            cdata->collision_info.collision_object_names.push_back(std::make_pair(first_object_name,second_object_name));
            cdata->collision_info.number_of_collisions++;

            //if( (!request.enable_cost) && (( result.numContacts() >= request.num_max_contacts) || (!request.enable_contact)) )
            if( (!request.enable_cost) && ( cdata->collision_info.stop_after_first_collision)  )
                cdata->done = true;
        }
        else
        {
            LOG_DEBUG_S<<"[defaultCollisionFunction]: There is no collision between "<<first_object_name.c_str()<<" and "<<second_object_name.c_str();
        }
    }
    else
    {
        LOG_DEBUG_S <<"[defaultCollisionFunction]: The collision between " <<first_object_name.c_str() <<" and " 
        <<second_object_name.c_str()<<"  ignored";
    }

    return cdata->done;
}


bool defaultDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double& dist)
{

    DistanceData* cdata                         = static_cast<DistanceData*>(cdata_);
    const fcl::DistanceRequest<double>& request = cdata->request;
    fcl::DistanceResult<double>& result         = cdata->result;
    dist = std::numeric_limits<double>::max();

    if(cdata->done)
    {
        dist = result.min_distance;
        return true;
    }
    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o1->getUserData());
    o2_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o2->getUserData());
    std::string first_object_name       = o1_collision_object_associated_data->getID();
    std::string second_object_name      = o2_collision_object_associated_data->getID();
    first_object_name=first_object_name.substr(0,first_object_name.find_last_of("_"))  ;
    second_object_name=second_object_name.substr(0,second_object_name.find_last_of("_"));
    
    LOG_DEBUG_S<<"[defaultDistanceFunction]: Checking collision between "<<first_object_name.c_str()<<" and " <<second_object_name.c_str();
    

    if(AbstractCollisionDetection::linksToBeChecked(first_object_name, second_object_name ))
    {
        // It seems broad_phase_collision_manager having problem with finding collision pair.
        // So one need to set high value for min_distance
        // See FCL issue #403: https://github.com/flexible-collision-library/fcl/issues/403
        result.min_distance = std::numeric_limits<double>::max(); 
        fcl::distance(o1, o2, request, result);

        if(result.min_distance > 0)
        {
            LOG_DEBUG_S<<"[defaultDistanceFunction]: Distance between " <<first_object_name.c_str() <<" and " 
            <<second_object_name.c_str()<<"  ="<<result.min_distance;
            if(request.enable_signed_distance)
            {
                DistanceInformation distance_information;
                distance_information.object1                = o1_collision_object_associated_data->getID(); //first_object_name;
                distance_information.object2                = o2_collision_object_associated_data->getID();//second_object_name;
                distance_information.min_distance           = result.min_distance;
                distance_information.nearest_points.at(0)   = result.nearest_points[0];
                distance_information.nearest_points.at(1)   = result.nearest_points[1];
                distance_information.contact_normal         =  (distance_information.nearest_points.at(0) - distance_information.nearest_points.at(1));
                // preventing divide by zero
                if(distance_information.contact_normal.x() == 0 && distance_information.contact_normal.y() == 0 && distance_information.contact_normal.z()  == 0)
                    distance_information.contact_normal +=  Eigen::Vector3d(1e-4, 1e-4, 1e-4);
                // Getting the normal vector need to be reworked
                // The current version of using contact point to calculate normal vector won't works
                // as contact point calculation depends on the object type(sphere, box ,mesh, octree etc.,)
                distance_information.contact_normal         =  distance_information.contact_normal / distance_information.contact_normal.norm();
                
                cdata->list_of_distance_information.push_back(distance_information);

//              cdata->collision_info.collision_cost += result.min_distance;
            }
        }
        else if( result.min_distance < 0) 
        {
            
             LOG_DEBUG_S<<"[defaultDistanceFunction]: There is collision between " <<first_object_name.c_str() 
            <<" and " <<second_object_name.c_str();
            cdata->collision_info.collision_object_names.push_back(std::make_pair(first_object_name,second_object_name));
            cdata->collision_info.number_of_collisions++;

            if (cdata->collision_info.stop_after_first_collision)  // stopping criteria in case of no detailed collision detail is needed.            
                cdata->done = true;
        }
    }
    return cdata->done;
}

bool completeDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double& dist)
{
    
    DistanceData* cdata                         = static_cast<DistanceData*>(cdata_);
    const fcl::DistanceRequest<double>& request = cdata->request;
    fcl::DistanceResult<double>& result         = cdata->result;
    dist = std::numeric_limits<double>::max();

    if(cdata->done)
    {
        dist = result.min_distance;
        return true;
    }

    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    o1_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o1->getUserData());    
    o2_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(o2->getUserData());
    std::string first_object_name       = o1_collision_object_associated_data->getID();
    std::string second_object_name      = o2_collision_object_associated_data->getID();
    first_object_name=first_object_name.substr(0,first_object_name.find_last_of("_"))  ;
    second_object_name=second_object_name.substr(0,second_object_name.find_last_of("_"));
    
    LOG_DEBUG_S<<"[defaultDistanceFunction]: Checking collision between "<<first_object_name.c_str()<<" and " <<second_object_name.c_str();

    if(AbstractCollisionDetection::linksToBeChecked(first_object_name, second_object_name ))
    {
        // It seems broad_phase_collision_manager having problem with finding collision pair.
        // So one need to set high value for min_distance
        // See FCL issue #403: https://github.com/flexible-collision-library/fcl/issues/403
        result.min_distance = std::numeric_limits<double>::max(); 
        fcl::distance(o1, o2, request, result);


        if(result.min_distance > 0)
        {
            LOG_DEBUG_S<<"[defaultDistanceFunction]: Distance between " <<first_object_name.c_str() <<" and " 
            <<second_object_name.c_str()<<"  ="<<result.min_distance;
            DistanceInformation distance_information;
            distance_information.object1                = o1_collision_object_associated_data->getID(); //first_object_name;
            distance_information.object2                = o2_collision_object_associated_data->getID();//second_object_name;
            distance_information.min_distance           = result.min_distance;
            distance_information.nearest_points.at(0)   = result.nearest_points[0];
            distance_information.nearest_points.at(1)   = result.nearest_points[1];
            distance_information.contact_normal         =  (distance_information.nearest_points.at(0) - distance_information.nearest_points.at(1));
            // preventing divide by zero
            if(distance_information.contact_normal.x() == 0 && distance_information.contact_normal.y() == 0 && distance_information.contact_normal.z()  == 0)
                distance_information.contact_normal +=  Eigen::Vector3d(1e-4, 1e-4, 1e-4);
            if(result.min_distance >= 0)
                distance_information.contact_normal         =  distance_information.contact_normal / distance_information.contact_normal.norm();
            else
                distance_information.contact_normal         =  distance_information.nearest_points.at(0);

            cdata->list_of_distance_information.push_back(distance_information);

        }
        else if( result.min_distance < 0) 
        {
             LOG_DEBUG_S<<"[defaultDistanceFunction]: There is collision between " <<first_object_name.c_str() 
            <<" and " <<second_object_name.c_str();

            fcl::CollisionRequest<double> request(100, true,1, true, true, fcl::GJKSolverType::GST_LIBCCD, 1e-3);
            fcl::CollisionResult<double> result;

            fcl::collide(o1, o2, request, result);
            std::vector<fcl::Contact<double>> fcl_collision_contacts;
            result.getContacts(fcl_collision_contacts);

            for(size_t i = 0; i<fcl_collision_contacts.size(); ++i)
            {
                const fcl::Contact<double> &cont = fcl_collision_contacts.at(i);

                DistanceInformation contact_info;
                contact_info.min_distance = -cont.penetration_depth;
                contact_info.nearest_points.at(0) = cont.pos;
                contact_info.nearest_points.at(1) = cont.pos;
                contact_info.contact_normal = cont.normal;

                contact_info.object1  = o1_collision_object_associated_data->getID();
                contact_info.object2 = o2_collision_object_associated_data->getID();
                 cdata->list_of_distance_information.push_back(contact_info);

            }

            cdata->collision_info.collision_object_names.push_back(std::make_pair(first_object_name,second_object_name));
            cdata->collision_info.number_of_collisions++;

        }
    }
    return cdata->done;
}
////////////////////// End of out of class variables and functions /////////////////////////////////////////////


FCLCollisionDetection::FCLCollisionDetection(CollisionDetectionConfig collision_detection_config): 
                                            collision_detection_config_(collision_detection_config)
{
    broad_phase_collision_manager.reset(new fcl::DynamicAABBTreeCollisionManager<double> );
    num_octree_ = 0;
}

FCLCollisionDetection::~FCLCollisionDetection()
{
    for(std::size_t i=0;i<collision_data_.size(); ++i)    
        delete collision_data_.at(i);        
}

bool FCLCollisionDetection::extractTrianglesAndVerticesFromMesh(const std::string &abs_path_to_mesh_file, std::vector<fcl::Triangle> &triangles, 
                                                                std::vector<fcl::Vector3d>& vertices, double scale_for_mesha_files_x=1.00, 
                                                                double scale_for_mesha_files_y=1.00, double scale_for_mesha_files_z=1.00 )
{
    Assimp::Importer assimp_importer;
    const aiScene * scene;

    //There are several way to read a cad file, available ways to read he cad file are:
    //aiProcess_Triangulate |aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_OptimizeGraph | aiProcess_OptimizeMeshes

    scene = assimp_importer.ReadFile(abs_path_to_mesh_file.c_str() , aiProcess_Triangulate);
    
    if(scene == NULL)
    {
        LOG_ERROR_S<<"[extractTrianglesAndVerticesFromMesh]: Assimp cannot read the given file";
        LOG_ERROR_S<<"The error message is "<<assimp_importer.GetErrorString();
        return false;
    }
 
    LOG_DEBUG_S<<"[extractTrianglesAndVerticesFromMesh]: Start extracting vertex and triangles from mesh file:" <<abs_path_to_mesh_file.c_str()
    <<".\nNumber of meshes found = "<<scene->mNumMeshes; 

    fcl::Vector3d vetex;
    fcl::Triangle triangle;

    for(std::size_t i=0; i<scene->mNumMeshes; ++i )
    {
        for(std::size_t j=0;j<scene->mMeshes[i]->mNumFaces; ++j)
        {
            triangle.set(scene->mMeshes[i]->mFaces[j].mIndices[0], scene->mMeshes[i]->mFaces[j].mIndices[1] , scene->mMeshes[i]->mFaces[j].mIndices[2]);
            triangles.push_back(triangle);
        }

        for(std::size_t j=0;j<scene->mMeshes[i]->mNumVertices; ++j)
        {
            //vetex.setValue(scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x, scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y, 
//scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z) ;
            vetex.x() = scene->mMeshes[i]->mVertices[j].x* scale_for_mesha_files_x;
            vetex.y() = scene->mMeshes[i]->mVertices[j].y*scale_for_mesha_files_y;
            vetex.z() = scene->mMeshes[i]->mVertices[j].z*scale_for_mesha_files_z;

            vertices.push_back(vetex);
        }
    }
//    delete scene;

    return true;
}

void FCLCollisionDetection::updateEnvironment(const std::shared_ptr<octomap::OcTree> &octomap, const std::string &env_object_name)
{
//     collision_objects_maps::iterator it=collision_objects_container_.find(env_object_name);
// 
//     if (it == collision_objects_container_.end())
//     {
//         LOG_WARN_S<<"[FCLCollisionDetection]: Trying to update environment object name "<<env_object_name.c_str()
//         <<". This object name is not available in collision_objects_container_";
//         return;
//     }
//     //octomap_ptr_->clear();
//     //octomap_ptr_ = std::move(octomap);
//     octomap_ptr_ = (octomap);    
//     std::cout<<"updateEnvironment in CC size = "<<octomap_ptr_->size()<<"   "<<octomap->size()<<std::endl;
//     broad_phase_collision_manager->update(it->second.get());
    

    
    ///////////////////
    // TODO: 
    // Simply copying the octomap is not updating the fcl based octree, so if new environment is updated, fcl fails to find collision.
    // So a temporary solution is to remove the old environment and then re-register the new environment. This quick fix is not an elegant solution,
    // a proper solution to this problem should be found to handle continuous input octomap.
    removeWorldCollisionObject(env_object_name);

    base::Pose collision_object_pose;
    collision_object_pose.position.setZero();
    collision_object_pose.orientation.setIdentity();
    
    registerOctreeToCollisionManager(octomap, collision_object_pose, env_object_name);
}

void FCLCollisionDetection::updateOctomapBoxesEnvironment(const std::shared_ptr<octomap::OcTree> &octomap, const std::string &env_object_name)
{

    removeOctomapBoxes(env_object_name);
    
    base::Pose collision_object_pose;
    collision_object_pose.position.setZero();
    collision_object_pose.orientation.setIdentity();
    
    registerOctreeAsBoxesToCollisionManager(octomap, collision_object_pose, env_object_name);                                                             
}


bool FCLCollisionDetection::removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size)
{
    if(octomap_ptr_)
    {
        double map_resoultion = octomap_ptr_->getResolution();
        double length = object_pose.x() + (object_size.x()/2.0);
        double width  = object_pose.y() + (object_size.y()/2.0);
        double height = object_pose.z() + (object_size.z()/2.0);

        for (double ix = (object_pose.x()-(object_size.x()/2.0)) ; ix <= length; ix += map_resoultion)
        {
            for (double iy = (object_pose.y()-(object_size.y()/2.0)); iy <= width; iy += map_resoultion)
            {
                for (double iz = (object_pose.z()-(object_size.z()/2.0)); iz <= height; iz += map_resoultion)
                {
                    // known cell
                    if (octomap_ptr_->search(ix,iy,iz))
                        octomap_ptr_->setNodeValue(ix,iy,iz, -1.0, false);
                }
            }
        }
        octomap_ptr_->updateInnerOccupancy();

        return true;
    }
    return false;
}

void FCLCollisionDetection::registerOctreeToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap, const base::Pose &collision_object_pose, 
                                                             std::string link_name)
{
    //octomap_ptr_ = std::move(octomap);
    octomap_ptr_ = octomap;
    
    LOG_DEBUG_S<<"[FCLCollisionDetection]: Registering octomap of size = "<<octomap_ptr_->size();

    //1) register octomap tree to fcl
    shared_ptr<fcl::OcTree<double>> fcl_OcTree_ptr(new fcl::OcTree<double>(  octomap_ptr_) );
    
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    fcl_OcTree_ptr->setUserData( collision_object_associated_data );
    
    
//    shared_ptr< fcl::CollisionObject<double> > fcl_tree_collision_object_ptr (new fcl::CollisionObject<double>( fcl_OcTree_ptr, 
//                                                                                                                collision_object_pose.orientation.toRotationMatrix(),
//                                                                                                                collision_object_pose.position ) ) ;
    //fcl_tree_collision_object_ptr_.reset(new fcl::CollisionObject<double>( fcl_OcTree_ptr));
    fcl_tree_collision_object_ptr_.reset(new fcl::CollisionObject<double>( fcl_OcTree_ptr, collision_object_pose.orientation.toRotationMatrix(), collision_object_pose.position ) ) ;
    
    registerCollisionObjectToCollisionManager(link_name, fcl_tree_collision_object_ptr_);
}


void FCLCollisionDetection::registerOctreeAsBoxesToCollisionManager(const std::shared_ptr<octomap::OcTree> &octomap, const base::Pose &collision_object_pose, 
                                                                    std::string link_name)
{

    LOG_DEBUG_S<<"[FCLCollisionDetection]: Registering octomap of size = "<<octomap->size();

    //1) register octomap tree to fcl
    shared_ptr<fcl::OcTree<double>> fcl_OcTree_ptr(new fcl::OcTree<double>(octomap->getResolution()));
    fcl_tree_collision_object_ptr_.reset(new fcl::CollisionObject<double>( fcl_OcTree_ptr));
    
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    fcl_OcTree_ptr->setUserData( collision_object_associated_data );
    collision_data_.push_back(collision_object_associated_data);
    
    
    fcl_octomap_boxes_.clear();
    fcl_octomap_boxes_.reserve(octomap->size()/2);

    double octree_thres = octomap->getOccupancyThres();
    for(auto it = octomap->begin(octomap->getTreeDepth()), end = octomap->end();it != end; ++it)
    {
        if(it->getOccupancy() >= octree_thres)
        {
            double box_size = it.getSize();
            fcl::Box<double>* box = new fcl::Box<double>(box_size, box_size, box_size);
            box->cost_density = (*it).getOccupancy();
            box->threshold_occupied = octomap->getOccupancyThres();
            fcl::CollisionObject<double>* obj = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(box), Eigen::Matrix3d::Identity(),
                                                                                 Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));

            CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
            collision_object_associated_data->setID(link_name);
            obj->setUserData( collision_object_associated_data );
            fcl_octomap_boxes_.push_back(obj);
        }
    }
// std::cout<<"registering octomax = "<<fcl_octomap_boxes_.size()<<std::endl;

    broad_phase_collision_manager->registerObjects(fcl_octomap_boxes_);

    CollisionObjectPair link_name_CollisionObject;
    link_name_CollisionObject.first  = link_name;
    link_name_CollisionObject.second = fcl_tree_collision_object_ptr_;
    collision_objects_container_.insert(link_name_CollisionObject );

}


bool FCLCollisionDetection::registerMeshToCollisionManager( const std::string &abs_path_to_mesh_file, const Eigen::Vector3d &mesh_scale, 
                                                            const std::string &link_name, const base::Pose &collision_object_pose, const double &link_padding)
{
    LOG_DEBUG_S<<"[FCLCollisionDetection]: Registering mesh file: "<<abs_path_to_mesh_file.c_str();

    std::vector<fcl::Triangle> triangles;
    std::vector<fcl::Vector3d> vertices;

    scale_mesh_(0) = mesh_scale(0) * link_padding;
    scale_mesh_(1) = mesh_scale(1) * link_padding;
    scale_mesh_(2) = mesh_scale(2) * link_padding;

    if(!extractTrianglesAndVerticesFromMesh(abs_path_to_mesh_file , triangles, vertices , scale_mesh_(0), scale_mesh_(1), scale_mesh_(2)))
        return false;

    registerMeshToCollisionManager(link_name, collision_object_pose, triangles, vertices);

    return true;
}

/*
link_name is collision_object_name, in fact we add  number to the end of link name to make a name for collision object name, 
wrist_1, wrist_2, wrist_3, wrist_4,.... 
*/

void FCLCollisionDetection::registerMeshToCollisionManager(const std::string &link_name, const base::Pose &collision_object_pose, 
                                                           const std::vector<fcl::Triangle> &triangles, const std::vector<fcl::Vector3d> &vertices)
{

    shared_ptr<fcl::BVHModel<fcl::OBBRSS<double> > >  fcl_mesh_ptr(new fcl::BVHModel<fcl::OBBRSS<double>>);
    fcl_mesh_ptr->beginModel();
    fcl_mesh_ptr->addSubModel(vertices,triangles);
    fcl_mesh_ptr->endModel();
    //fcl::Transform3d mesh_transform3f(collision_object_quaternion_orientation,collision_object_translation);
    //shared_ptr<fcl::CollisionObject<double>>   mesh_collision_object ( new fcl::CollisionObject<double>( fcl_mesh_ptr , mesh_transform3f )  );
    shared_ptr<fcl::CollisionObject<double>> mesh_collision_object_ptr ( new fcl::CollisionObject<double>( fcl_mesh_ptr, 
                                                                        collision_object_pose.orientation.toRotationMatrix(), collision_object_pose.position ) );

    registerCollisionObjectToCollisionManager(link_name, mesh_collision_object_ptr); 

}

void FCLCollisionDetection::registerBoxToCollisionManager(const double &box_x, const double &box_y, const double &box_z, const std::string &link_name ,
                                                          const base::Pose &collision_object_pose, const double &link_padding )

{
    shared_ptr<fcl::Box<double> > fcl_box_ptr(new fcl::Box<double>(box_x*link_padding, box_y*link_padding, box_z*link_padding));
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    fcl_box_ptr->setUserData( collision_object_associated_data );
    
    shared_ptr< fcl::CollisionObject<double>   > box_collision_object_ptr (new fcl::CollisionObject<double>( fcl_box_ptr, 
                                                                            collision_object_pose.orientation.toRotationMatrix(), 
                                                                            collision_object_pose.position ) ) ;

    registerCollisionObjectToCollisionManager(link_name, box_collision_object_ptr);  

}

void FCLCollisionDetection::registerCylinderToCollisionManager(const double &radius, const double &length, const std::string &link_name ,
                                                               const base::Pose &collision_object_pose, const double &link_padding )
{
    shared_ptr<fcl::Cylinder<double> > fcl_cylinder_ptr(new fcl::Cylinder<double>(radius*link_padding,length*link_padding));
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    fcl_cylinder_ptr->setUserData( collision_object_associated_data );

    shared_ptr< fcl::CollisionObject<double>   > cylinder_collision_object_ptr (new fcl::CollisionObject<double>( fcl_cylinder_ptr, 
                                                                                  collision_object_pose.orientation.toRotationMatrix(), 
                                                                                  collision_object_pose.position ) ) ;

    registerCollisionObjectToCollisionManager(link_name, cylinder_collision_object_ptr);  

}

void FCLCollisionDetection::registerSphereToCollisionManager(const double &radius, const std::string &link_name , const base::Pose &collision_object_pose, const double &link_padding )
{
    shared_ptr<fcl::Sphere<double> > fcl_sphere_ptr(new fcl::Sphere<double> (radius*link_padding));
    
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    fcl_sphere_ptr->setUserData( collision_object_associated_data );

    shared_ptr< fcl::CollisionObject<double>   > sphere_collision_object_ptr (new fcl::CollisionObject<double>( fcl_sphere_ptr, 
                                                                                collision_object_pose.orientation.toRotationMatrix(), 
                                                                                collision_object_pose.position  ) ) ;

    registerCollisionObjectToCollisionManager(link_name, sphere_collision_object_ptr);
}

void FCLCollisionDetection::registerCollisionObjectToCollisionManager(const std::string &link_name, shared_ptr< fcl::CollisionObject<double> > &collision_object )
{
    CollisionObjectAssociatedData *collision_object_associated_data(new CollisionObjectAssociatedData );
    collision_object_associated_data->setID(link_name);
    collision_object->setUserData( collision_object_associated_data );
    collision_data_.push_back(collision_object_associated_data);

    broad_phase_collision_manager->registerObject(collision_object.get());

    CollisionObjectPair link_name_CollisionObject;
    link_name_CollisionObject.first  = link_name;
    link_name_CollisionObject.second = collision_object;
    collision_objects_container_.insert(link_name_CollisionObject );
}

bool FCLCollisionDetection::removeSelfCollisionObject(const std::string &collision_object_name)
{
    //find the collision object -
    CollisionObjectsMap::iterator it=collision_objects_container_.find(collision_object_name);

    if(it == collision_objects_container_.end())
    {
        LOG_WARN_S<<"[FCLCollisionDetection]: Trying to remove object name "<<(collision_object_name).c_str()
        <<". This object name is not available in collision_objects_container_";	
        return false;
    }

    //unregister the collision object from collision manager.
    broad_phase_collision_manager->unregisterObject(it->second.get());

    //remove collision object from collision containter
    removeObject4mCollisionContainer(collision_object_name);

    return true;
}

bool FCLCollisionDetection::removeWorldCollisionObject(const std::string &collision_object_name)
{
    //find the collision object -
    CollisionObjectsMap::iterator it = collision_objects_container_.find(collision_object_name);

    if (it == collision_objects_container_.end())
    {
        LOG_WARN_S<<"[FCLCollisionDetection]: Trying to remove object name "<<collision_object_name.c_str()
        <<" from the world. This object name is not available in collision_objects_container_";
        return false;
    }
 
    //unregister the collision object from collision manager.
    broad_phase_collision_manager->unregisterObject(it->second.get());

    //remove collision object from collision containter
    removeObject4mCollisionContainer(collision_object_name);

    return true;
}

bool FCLCollisionDetection::removeOctomapBoxes(const std::string &collision_object_name)
{
    //find the collision object -
    CollisionObjectsMap::iterator it = collision_objects_container_.find(collision_object_name);

    if (it == collision_objects_container_.end())
    {
        LOG_WARN_S<<"[FCLCollisionDetection]: Trying to remove object name "<<collision_object_name.c_str()
        <<" from the world. This object name is not available in collision_objects_container_";
        return false;
    }
 
    //unregister the collision object from collision manager.
    for(std::size_t j = 0; j < fcl_octomap_boxes_.size(); ++j)
    {
        broad_phase_collision_manager->unregisterObject(fcl_octomap_boxes_[j]);
        delete fcl_octomap_boxes_[j];
    }

    //remove collision object from collision containter
    removeObject4mCollisionContainer(collision_object_name);

    return true;
}

void FCLCollisionDetection::removeObject4mCollisionContainer(const std::string &collision_object_name)
{
    remove_collision_object_4m_collisionManager = collision_object_name;

    collision_data_.erase(std::remove_if(collision_data_.begin(), collision_data_.end(),
                                        isObjectListedInCollisionObjectAssociatedData), collision_data_.end());

    //remove the collision object from collisionObjects container
    collision_objects_container_.erase(collision_object_name);
}


void FCLCollisionDetection::updateCollisionObjectTransform(std::string link_name, const base::Pose collision_object_pose)
{
    CollisionObjectsMap::iterator it=collision_objects_container_.find(link_name);

    if (it == collision_objects_container_.end())
    {
        LOG_WARN_S<<"[FCLCollisionDetection]: Trying to update collision object name "<<link_name
        <<". This object name is not available in collision_objects_container_";	   

        return;
    }

    it->second->setTransform(collision_object_pose.orientation, collision_object_pose.position);
    it->second->computeAABB();
    broad_phase_collision_manager->update(it->second.get());
}


// We are going to downcast the abstract collision object
bool FCLCollisionDetection::assignWorldDetector(AbstractCollisionPtr collision_detector)
{
    try
    {
        world_collision_detector_.reset(new FCLCollisionDetection(collision_detection_config_));
        world_collision_detector_ = dynamic_pointer_cast<FCLCollisionDetection>(collision_detector);
        //world_collision_detector_ = collision_detector;
    }
    catch (std::exception &e)
    {
        LOG_ERROR("[FCLCollisionDetection]: Cannot assign AbstractCollisionPtr to FCLCollisionDetection. The error:%s", e.what());
        return false;
    }

    return true;
}

void FCLCollisionDetection::calculateCompleteDistanceInfo()
{
    collision_object_names_.clear();
    full_collision_distance_information_.clear();

    // First check self collision
    DistanceData self_collision_data = getDistanceData();
    broad_phase_collision_manager->distance(&self_collision_data, completeDistanceFunction);
    full_collision_distance_information_ = self_collision_data.list_of_distance_information; // store the distance information

    // Now we do collision check withe environment
    DistanceData env_collision_data;
    broad_phase_collision_manager->distance( world_collision_detector_->getCollisionManager().get(), &env_collision_data, completeDistanceFunction);

    // store the distance information
    full_collision_distance_information_.insert(full_collision_distance_information_.end(), env_collision_data.list_of_distance_information.begin(),
                                            env_collision_data.list_of_distance_information.end());

}

void FCLCollisionDetection::calculateOnlyEnvironmentDistanceInfo()
{
    collision_object_names_.clear();
    full_collision_distance_information_.clear();    

    // Collision check with the environment
    DistanceData env_collision_data = getDistanceData();
    broad_phase_collision_manager->distance( world_collision_detector_->getCollisionManager().get(), &env_collision_data, completeDistanceFunction);
    full_collision_distance_information_ = env_collision_data.list_of_distance_information; // store the distance information
}


bool FCLCollisionDetection::isCollisionsOccured( double &total_cost)
{
    total_cost = 0.0;
    collision_object_names_.clear();
    env_collision_distance_information_.clear();
    full_collision_distance_information_.clear();
    
    // First check self collision
    switch (collision_detection_config_.collision_info_type)
    {
        case collision_detection::DISTANCE:
        {
            DistanceData self_collision_data = getDistanceData();
            broad_phase_collision_manager->distance(&self_collision_data, defaultDistanceFunction); 
            if (self_collision_data.collision_info.number_of_collisions > 0)
            {
                total_cost = self_collision_data.collision_info.collision_cost;
                // std::cout<<"Self Collision total Cost = "<< total_cost<<std::endl;
                collision_object_names_ = self_collision_data.collision_info.collision_object_names;
                full_collision_distance_information_ = self_collision_data.list_of_distance_information; // store the distance information
                if ( collision_detection_config_.stop_after_first_collision)  //special condition. Explained in the variable definition
                    return true;

                LOG_DEBUG("[FCLCollisionDetection]: There are self collisions, now we are checking for collisions against environment");
            }
            else
                LOG_DEBUG("[FCLCollisionDetection]: There is no self collisions, now we are checking for collisions against environment");

            // Now we do collision check withe environment
            DistanceData env_collision_data = getDistanceData();
            broad_phase_collision_manager->distance(  world_collision_detector_->getCollisionManager().get(), &env_collision_data, defaultDistanceFunction);
            if (env_collision_data.collision_info.number_of_collisions > 0)
            {
                collision_object_names_.insert(collision_object_names_.end(), env_collision_data.collision_info.collision_object_names.begin(),
                                               env_collision_data.collision_info.collision_object_names.end());
                if ( collision_detection_config_.stop_after_first_collision)
                    return true;

                total_cost += env_collision_data.collision_info.collision_cost;
                 //std::cout<<"Env Collision total Cost ="<< total_cost<<"  Env cost only ="<<env_collision_data.collision_info.collision_cost<<std::endl;
                
                // store the distance information
                full_collision_distance_information_.insert(full_collision_distance_information_.end(), env_collision_data.list_of_distance_information.begin(),
                                                        env_collision_data.list_of_distance_information.end());
                return true;
            }
            else
                LOG_DEBUG("[FCLCollisionDetection]: There is no collisions against environment" );
            
            if (self_collision_data.collision_info.number_of_collisions > 0)
                return true;

            break;
        }
        case collision_detection::MULTI_CONTACT:
        {
            CollisionData self_collision_data = getCollisionData();
            broad_phase_collision_manager->collide(&self_collision_data, defaultCollisionFunction);
            if (self_collision_data.collision_info.number_of_collisions > 0)
            {

                collision_object_names_ = self_collision_data.collision_info.collision_object_names;
                if ( collision_detection_config_.stop_after_first_collision)
                    return true;
                
                total_cost = getCollisionCost(self_collision_data, full_collision_distance_information_); // store the distance information

                // std::cout<<"Self Collision total Cost = "<< total_cost<<std::endl;
                LOG_DEBUG("[FCLCollisionDetection]: There are self collisions, now we are checking for collisions against environment");
            }
            else
                LOG_DEBUG("[FCLCollisionDetection]: There is no self collisions, now we are checking for collisions against environment");
            // Now we do collision check withe environment
            CollisionData env_collision_data = getCollisionData();
            broad_phase_collision_manager->collide(  world_collision_detector_->getCollisionManager().get(), &env_collision_data, defaultCollisionFunction);
            if (env_collision_data.collision_info.number_of_collisions > 0)
            {
                collision_object_names_.insert(collision_object_names_.end(), env_collision_data.collision_info.collision_object_names.begin(),
                                               env_collision_data.collision_info.collision_object_names.end());

                if ( collision_detection_config_.stop_after_first_collision)
                    return true;

                total_cost += getCollisionCost(env_collision_data, env_collision_distance_information_); // store the distance information 
                // store the distance information
                full_collision_distance_information_.insert(full_collision_distance_information_.end(), env_collision_distance_information_.begin(),
                                                            env_collision_distance_information_.end());
                
                return true;
            }
            else
                LOG_DEBUG("[FCLCollisionDetection]: There is no collisions against environment" );

            if (self_collision_data.collision_info.number_of_collisions > 0)
                return true;
            
            break;

        }
        default:
            LOG_ERROR("[FCLCollisionDetection]: Unknown collision info requested");
            break;
    }
    
    return false;
}


/**/
bool FCLCollisionDetection::distanceOfClosestObstacleToRobot(shared_ptr<fcl::BroadPhaseCollisionManager<double>> &external_broad_phase_collision_manager,DistanceData &distance_data)
{
    this->broad_phase_collision_manager->distance( external_broad_phase_collision_manager.get(), &distance_data, defaultDistanceFunction);

    if(distance_data.result.min_distance == 0)
        return false;// collison or touch    
    else    
        return true; // everything is fine
    
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


DistanceData FCLCollisionDetection::getDistanceData()
{
    DistanceData distance_data;
    
    distance_data.collision_info.number_of_collisions       = 0;
    distance_data.collision_info.stop_after_first_collision = collision_detection_config_.stop_after_first_collision;

    // If "max_num_collision_contacts" holds a value that it is more that one means, we need to calculate signed distance
    if(collision_detection_config_.calculate_distance_information)
    {
        distance_data.request.enable_signed_distance            = true;
        distance_data.request.enable_nearest_points             = true;
    }
    else
    {
        distance_data.request.enable_signed_distance            = false;
        distance_data.request.enable_nearest_points             = false;
    }

    return distance_data;
}

CollisionData FCLCollisionDetection::getCollisionData()
{
    CollisionData collision_data;
    
    collision_data.request.num_max_contacts                     = collision_detection_config_.max_num_collision_contacts;
    collision_data.collision_info.number_of_collisions          = 0;
    collision_data.collision_info.stop_after_first_collision    = collision_detection_config_.stop_after_first_collision;
    
    // If "max_num_collision_contacts" holds a value that it is more that one means, we need to calculate collision contact
    if(collision_detection_config_.calculate_distance_information)
        collision_data.request.enable_contact = true;
    else
        collision_data.request.enable_contact = false;

    return collision_data;
}

double FCLCollisionDetection::getCollisionCost(CollisionData &collision_data, std::vector<DistanceInformation> &contacts)
{
    double collision_cost = 0.0;
    std::vector<fcl::Contact<double>> fcl_collision_contacts;
    collision_data.result.getContacts(fcl_collision_contacts);

    contacts.resize(fcl_collision_contacts.size());
    
    CollisionObjectAssociatedData * o1_collision_object_associated_data, * o2_collision_object_associated_data;
    for(size_t i = 0; i<fcl_collision_contacts.size(); ++i)
    {
        const fcl::Contact<double> &cont = fcl_collision_contacts.at(i);

        DistanceInformation contact_info;
        contact_info.min_distance = cont.penetration_depth;
        contact_info.nearest_points.at(0) = cont.pos;
        contact_info.nearest_points.at(1) = cont.pos;
        contact_info.contact_normal = cont.normal;

        o1_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(cont.o1->getUserData());
        o2_collision_object_associated_data = static_cast<CollisionObjectAssociatedData*>(cont.o2->getUserData());

        // It seems that the mesh filename cannot be obtained using "o1_collision_object_associated_data->getID()"
        // So currently the collision obj name is obtained by collisionData
        try
        {
            contact_info.object1  = o1_collision_object_associated_data->getID();
            contact_info.object2 = o2_collision_object_associated_data->getID();       
        }
        catch (std::exception &e)
        {
            LOG_WARN("[getCollisionCost]: Cannot able to retrieve the collision object name. This error occurs if the collision objects are MESH files. Currently setting the collision object name to empty string.");
            contact_info.object1  = "";
            contact_info.object2 = "";
        }

        contacts.at(i) = contact_info;
        collision_cost +=  cont.penetration_depth;
    }
    return collision_cost;
}

std::vector< DistanceInformation>& FCLCollisionDetection::getCollisionDistanceInformation()
{
    return full_collision_distance_information_;
}

std::vector< DistanceInformation>& FCLCollisionDetection::getCompleteDistanceInformation()
{
    calculateCompleteDistanceInfo();
    return full_collision_distance_information_;
}

std::vector< DistanceInformation>& FCLCollisionDetection::getOnlyEnvironmentDistanceInformation()
{
    calculateOnlyEnvironmentDistanceInfo();
    return full_collision_distance_information_;
}

void FCLCollisionDetection::printCollisionObject()
{
    std::cout<<"The collision object containter contains "<<collision_objects_container_.size()<<" objects with the following names "<<std::endl;

    for(CollisionObjectsMap::iterator it = collision_objects_container_.begin(); it!=collision_objects_container_.end(); ++it)
    {
        std::cout<<it->first<<std::endl;
    }    
    //std::cout<<"---- End of print collision object funtion ------ "<<std::endl;
}

std::vector<std::string> FCLCollisionDetection::getRobotCollisionObjectsNames()
{
    std::vector<std::string> collision_object_names;

    for(CollisionObjectsMap::iterator it = collision_objects_container_.begin(); it!=collision_objects_container_.end(); ++it)
        collision_object_names.push_back(it->first);

    return collision_object_names;
}

std::vector<std::string> FCLCollisionDetection::getWorldCollisionObjectsNames()
{
    std::vector<std::string> collision_object_names;

    for(CollisionObjectsMap::iterator it = world_collision_detector_->collision_objects_container_.begin(); it!= world_collision_detector_->collision_objects_container_.end(); ++it)
        collision_object_names.push_back(it->first);

    return collision_object_names;
}
        
        

std::vector< std::pair<std::string, std::string> > FCLCollisionDetection::getCollidedObjectsNames()
{
   return collision_object_names_;
}

void FCLCollisionDetection::saveOctree()
{
    mkdir(collision_detection_config_.env_debug_config.save_octree_path.c_str(), 0755);     
    std::stringstream ss;
    ss<<collision_detection_config_.env_debug_config.save_octree_path <<"/"<<collision_detection_config_.env_debug_config.save_octree_filename<<"_"<<num_octree_<<".bt";
    
    octomap_ptr_->writeBinary(ss.str());
    num_octree_++;
}

}// end namespace collision_detection


