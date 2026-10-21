# header.stamp=扫描末，cloud=body_frame局部云，odom_pose=T_OB。
std_msgs/Header header
string session_id
uint64 id
geometry_msgs/Pose odom_pose
sensor_msgs/PointCloud2 cloud
