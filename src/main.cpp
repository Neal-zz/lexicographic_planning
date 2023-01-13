#include "pathPlanning.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "lexicographic_planning");
    pathPlanning os;
    ROS_INFO("\033[1;32m----> lexicographic_planning: Obstacle Server Started.\033[0m");
    ros::Rate rate(1);
    while (ros::ok()) {
        os.cloudHandler();
        os.pathHandler();
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}