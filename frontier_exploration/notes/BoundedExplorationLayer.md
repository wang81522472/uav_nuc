## Class: BoundedExploreLayer : Costmap_2d::Layer,Costmap_2d::Costmap2D
是costmap2D的plugin?
## 成员变量
- dynamic_reconfigure::Server *dsrv_   
- ros::ServiceServer polygonService_  
- ros::ServiceServer frontierService_  
- geometry_msgs::Polygon polygon_  
- tf::TransformListener tf_listener_  
- ros::Publisher frontier_cloud_pub
- bool configured_
- bool marked_
- std::string frontier_travel_point_
- bool resize_to_boundary_

##  成员函数

####构造函数BoundedExploreLayer()
空构造函数

#### ~BoundedExploreLayer()


#### 1. virtual void onInitialize()
@brief: Loads default values, 初始化exploration costmap.  
多态自layer.h，原函数为空。  
获取一些参数，根据参数进行初始化：  
发布一个.../frontiers的topic，初始化updateBoundaryPolygonService和getNextFrontierService。  
初始化dynamic reconfigure，绑定回调。

#### 2. virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x, double* polygon_max_y);
@brief Calculate bounds of costmap window to update  

#### 3. virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

#### 3. virtual void matchSize();
@brief Match dimensions and origin of parent costmap  

#### 4. virtual void reset();

#### 5. bool updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req,frontier_exploration::UpdateBoundaryPolygon::Response &res);

#### 6. bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);

#### 7. bool getNextFrontierService(frontier_exploration::UpdateBoundaryPolygon::Request &req,frontier_exploration::UpdateBoundaryPolygon::Response &res);

#### 8. bool getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);
找一个最近的
#### 9. void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

#### 10. void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);