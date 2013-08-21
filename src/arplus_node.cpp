#include <arplus_node.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "arplus_pose");    
    ros::NodeHandle n;
    ARPlus_Node node(n);
    ros::spin();
    return 0;
}
