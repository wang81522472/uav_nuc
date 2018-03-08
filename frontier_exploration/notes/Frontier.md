#Frontier.msg

uint32 size //有多少个Frontier Cell  
float64 min_distance //Robot离Frontier的最短距离  
geometry_msgs/Point initial //开始build的起点  
geometry_msgs/Point centroid //Frontier Cell的x,y的平均值  
geometry_msgs/Point middle //距离Robot最近的Frontier Cell  
