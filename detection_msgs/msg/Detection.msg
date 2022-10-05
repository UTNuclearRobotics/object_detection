# Class of object. ex: car, truck, suitcase, barrel...
string object_class

# Numeric tracking id of object. ex: If 2 barrels have already been seen the next object detected (regardless of class) will be given an id of 3.
uint16 object_id

# Pose of detected object
geometry_msgs/PoseStamped pose

# Point cloud of object detected
sensor_msgs/PointCloud2 cloud

# Compressed image of detected object
sensor_msgs/CompressedImage cmpr_image