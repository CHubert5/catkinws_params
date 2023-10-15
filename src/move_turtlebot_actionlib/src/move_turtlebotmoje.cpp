#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <iostream>
#include <limits>
#include <tf/transform_datatypes.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isGoalReached = true; // Global flag indicating whether the goal is reached

void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached");
        isGoalReached = true;
    } else {
        ROS_INFO("Failed to reach the goal");
        isGoalReached = false;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_turtlebot");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server");
    }

    std::vector<std::tuple<float, float, float>> coordinatesList;

    while (ros::ok()) {
        float pos_x, pos_y, orientation_deg;

        std::cout << "Enter the x position: ";
        std::cin >> pos_x;

        std::cout << "Enter the y position: ";
        std::cin >> pos_y;

        std::cout << "Enter orientation in degrees (-180 to 180): ";
        std::cin >> orientation_deg;

        if (std::cin.fail()) {
            ROS_ERROR("Input data error. Please try again.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        if (orientation_deg < -180.0 || orientation_deg > 180.0) {
            ROS_ERROR("Invalid orientation. Orientation should be in the range of -180 to 180 degrees.");
            continue;
        }

        double orientation_rad = orientation_deg * M_PI / 180.0;

        coordinatesList.push_back(std::make_tuple(pos_x, pos_y, orientation_rad));

        std::string input;
        std::cout << "To add more coordinates, enter 'n' and press Enter. To start navigation, enter 'start' and press Enter. To exit the program, enter 'exit' and press Enter: ";
        std::cin >> input;

        if (input == "n") {
            continue;
        } else if (input == "start") {
            ROS_INFO("Starting navigation to the specified coordinates.");

            for (const auto& coordinates : coordinatesList) {
                float x, y, orientation;
                std::tie(x, y, orientation) = coordinates;

                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = x;
                goal.target_pose.pose.position.y = y;

                tf::Quaternion quat;
                quat.setRPY(0, 0, orientation);
                tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);

                ROS_INFO("Sending goal");
                ac.sendGoal(goal, &goalDoneCallback);
                isGoalReached = false;

                while (!isGoalReached) {
                    ros::spinOnce();
                }
            }

            ROS_INFO("Navigation completed.");
            coordinatesList.clear();
            std::cout << "To exit the program, enter 'exit' and press Enter: ";
            std::cin >> input;
            if (input == "exit") {
                ROS_INFO("Program exited.");
                return 0;
            }
        } else if (input == "exit") {
            ROS_INFO("Exiting the program.");
            return 0;
        } else {
            ROS_ERROR("Invalid option. Please choose 'n', 'start', or 'exit'.");
        }
    }

    return 0;
}
