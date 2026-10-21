# 与每个已选Odometry同stamp并覆盖扫描末cloud_registered；header.frame_id标记odom参考系。
# odom原点重建/节点重启时更换，不能将不同会话的局部云拼为同一地图。
std_msgs/Header header
string session_id
