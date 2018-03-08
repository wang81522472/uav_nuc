# Coords of frontier cell
geometry_msgs/Point frontier_cell_point

# How many frontier cells can be seen
float64 information_gain

# Traverse cost from current location to frontier cell of ugv
float64 traverse_cost_ground

# Traverse cost from current location to frontier cell of uav
float64 traverse_cost_air

# How many frontier cells in frontier
uint32 frontier_size

# Utility of frontier cell for coordination
float32 utility

# Score of this frontier cell
float64 score

