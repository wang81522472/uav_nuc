#Class FrontierExplorationServer

负责FrontierExploration Task, 移动机器人

**整理**：此类包含一个构造函数（负责初始化actionlib, nh等）,  
还包含actionlib的回调函数（server的Execute, Preempt; client的Feedback, Done）  
Server类具有一个ActionlibServer（执行探索进程）和一个ActionlibClient(负责与move_base交互)

##成员变量
ros::NodeHandle nh_  
ros::NodeHandle private_nh_  
tf::TransformListener tf_listener_  

actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_  
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_  
boost::mutex move_client_lock_  

move_base_msgs::MoveBaseGoal move_client_goal_ // goal_pose的容器，使用move_client_发送出去  

boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_   

double frequency_  
double goal_aliasing_  
bool success_    
bool moving_  
int retry_  

frontier_exploration::ExploreTaskFeedback feedback_  

##成员函数
#### 0. 构造函数FrontierExplorationServer(std::string name)
只有一个Public方法，就是构造函数  
用初始化成员列表给tf_listener, private_nh_, as_, move_client_, retry_赋值  
初始化private_nh_的参数   
初始化explore_costmap_ros_  
初始化as_(绑定回调函数，start())

#### 1. void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
@brief: as_的Execute回调，在收到一个新Goal区域之后调用，负责调动机器人探索Goal  
@param: goal包含goal区域的boundary与centerpoint

**函数执行一次 代表对区域完整的搜索**

设置flag变量：sucess_为false，moving_为false

private_nh_绑定两个Service Client: UpdateBoundaryPolygon, GetNextFrontier  
等待move_base(actionlib的client)连接server(不清除在哪，估计由机器人发布？), 等待两个Service连接Server(在bounded_explore_layer.cpp里)  
在costmap上设定Goal里的boundary（用BoundedExploreLayer）  

进while循环，直到搜索完所有Frontier，while循环里面是：  
新建变量，用来存储goal_pose与机器人的pose，并评估pose是不是在goal里，若不是则将goal_pose设为Goal的中心，若是则设置为getNextFrontier，getNextFrontier失败则：之前找到过Frontier就退出，之前没找到过就retry，retry超过5次就退出。  
如果（机器人没在动，且goal_pose与老goal_pose(move_client_goal_.target_pose)离的足够远），则更新move_client_goal_, lock住move_client_lock_，用move_client_发送move_client_goal_，**绑定两个回调函数**，将moving_设置为true，最后unlock  
sleep，直到机器人完成动作




#### 2. void preemptCb()
@brief: as_的Preempt回调，取消正在running的goal与所有相关的action

#### 3. void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
@brief: move_base client的Feedback回调    
重新从as_发布  

#### 4. void doneMovingCb(const actionlib::SimpleClientGoalState& state, const   move_base_msgs::MoveBaseResultConstPtr& result)  
@brief: move_base client的Done回调  
@param: move_base client的state  
@param: move_base client的result  
如果state为ABORTED，则终止Explore  
如果state为SUCCEEDED，则moving_设置为false  