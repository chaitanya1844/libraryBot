# A candidate empty slot on a shelf detected from the depth camera
# point cloud by the shelf-inference node.

geometry_msgs/PoseStamped pose   # slot pose in the camera/base frame (as published)
float64 width                     # estimated free width of the slot (m)
float64 confidence                # 0-1 confidence the slot is real/empty
string shelf_id                   # shelf this slot belongs to
int32 shelf_level                 # 0 = bottom shelf level, increasing upward
