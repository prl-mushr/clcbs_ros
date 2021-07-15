#include "clcbs_ros/clcbs_node.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "clcbs_node");
    ros::NodeHandle nh;

    clcbs_ros::CLCBSNode node(nh);

    ros::spin();
    return 0;
}
