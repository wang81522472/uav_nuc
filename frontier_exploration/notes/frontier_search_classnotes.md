#Class: FrontierSearch
此类的功能是，Search for Frontier
###成员变量

1. costmap_
2. map_ (unsigned char *)
3. size_x_ (int)
4. size_y_ (int)

###成员函数
#####1. `FrontierSearch(costmap::Costmap2D & costmap)`
通过初始化成员列表将costmap赋给costmap_

#####2. `std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)`
检查position在不在地图内  
找到离position最近的clear cell  
从clear cell开始，使用广度优先搜索，用`buildNewFrontier`搜索出所有新的Frontier  
返回frontier_list

#####3. `Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag)`
初始化Frontier结构（包括centroid，size，min_distance）  
从InitialCell开始找新的FrontierCell（向周围8个cell蔓延），不断更新Frontier的数据

#####4. `bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag)`
查看Cell是unknown或者未标为frontier_cell
查找周围4个cell有没有Free Cell，如果是，则是Frontier Cell

