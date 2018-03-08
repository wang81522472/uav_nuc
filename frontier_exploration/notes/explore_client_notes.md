# Class: FrontierExplorationClient
负责从rviz中接收control points，并以之生成boundary polygon  

建立一个Subscriber，接受rviz中接收到的点；又建立一个Publisher来发布多边形的Marker，每隔一段时间（使用ros::WallDuration，似乎是每隔0.1s?）触发一次。
##成员变量
ros::NodeHandle nh_  
ros::NodeHandle private_nh_  

ros::Subscriber point_  
ros::Publisher point_viz_pub_  
ros::WallTimer point_viz_timer_  

geometry_msgs::PolygonStamped input_  

bool waiting_for_center_  

##成员函数
####构造函数FrontierExplorationClient()
初始化成员列表：nh_(), private_nh_("~"), waiting_for_center_(false)    
设置input_的frame_id为"map"   
订阅点击的点，绑定回调pointCb：topic /clicked_points    
发布Marker ：topic exploration_polygon_marker  
初始化point_viz_timer_，绑定回调vizPubCb  

#### 1. vizPubCb()
@brief: Publish多边形的marker  

这个函数没仔细看，基本上是把input_中已经添加了的points和连线发布出来

#### 2. pointCb()
@brief: 用接收到的点创建多边形
@param: rviz中接收到的点

如果wait_for_center_ = true（即已经收到了一个多边形，等待接收起始位置），并接收到了一个点，则：  
检查点在不在多边形里  
把center和polygon包装成goal，发送给explore_server节点，节点会触发executeCb（见explore_server_notes.md）  


如果input_空，说明这是多边形第一个点，则把input_.header = point.header，并且把值push进input_  

如果input_与point的frame_id不同，则报错，并清空input_

如果input_里至少两个点，且新的点（还没push进input_）与input_的第一个点离的近（用pointsNearby()函数），则：  
如果input_点数小于3，则报错（似乎不支持三角形？），若不是，则等待接受center点（手动选择的机器人初始位置），wait_for_center_ = true

