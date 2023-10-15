#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <iostream>
#include <limits>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool isGoalReached = true; // Globalna flaga informująca, czy osiągnięto cel

void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Osiągnięto cel");
    } else {
        ROS_INFO("Nie udało się osiągnąć celu");
    }
    isGoalReached = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_turtlebot");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Oczekiwanie na serwer akcji move_base");
    }

    std::vector<move_base_msgs::MoveBaseGoal> goals; // Lista celów nawigacyjnych

    while (ros::ok()) {
        float pos_x, pos_y, orientation_deg;

        std::cout << "Podaj pozycję x: ";
        std::cin >> pos_x;

        std::cout << "Podaj pozycję y: ";
        std::cin >> pos_y;

        std::cout << "Podaj orientację w stopniach (-180 do 180): ";
        std::cin >> orientation_deg;

        if (std::cin.fail()) {
            ROS_ERROR("Błąd w danych wejściowych. Spróbuj ponownie.");
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        // Upewnij się, że orientacja jest w zakresie od -180 do 180 stopni
        if (orientation_deg < -180.0 || orientation_deg > 180.0) {
            ROS_ERROR("Nieprawidłowa orientacja. Orientacja powinna być w zakresie od -180 do 180 stopni.");
            continue;
        }

        // Przelicz orientację z stopni na radiany
        double orientation_rad = orientation_deg * M_PI / 180.0;

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = pos_x;
        goal.target_pose.pose.position.y = pos_y;

        // Ustaw orientację w wiadomości
        tf::Quaternion quat;
        quat.setRPY(0, 0, orientation_rad); // Roll, Pitch, Yaw
        tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);

        goals.push_back(goal);

        std::string input;
        std::cout << "Aby dodać kolejne cele, wpisz 'n' i naciśnij Enter. Aby rozpocząć nawigację, wpisz 'start' i naciśnij Enter. Aby zakończyć program, wpisz 'exit' i naciśnij Enter: ";
        std::cin >> input;

        if (input == "n") {
            continue;
        } else if (input == "start") {
            ROS_INFO("Rozpoczynam nawigację do określonych celów.");

            for (const auto& goal : goals) {
                ROS_INFO("Wysyłanie celu");
                ac.sendGoal(goal, &goalDoneCallback);
                isGoalReached = false;

                while (!isGoalReached) {
                    // Oczekiwanie na osiągnięcie celu lub wpisanie "q" do przerwania akcji
                    std::cin >> input;
                    if (input == "q") {
                        ROS_INFO("Przerwano nawigację.");
                        ac.cancelGoal();
                        break;
                    }
                    ros::spinOnce();
                }
            }

            ROS_INFO("Nawigacja zakończona.");
            goals.clear();
        } else if (input == "exit") {
            ROS_INFO("Zakończono program.");
            return 0;
        } else {
            ROS_ERROR("Nieprawidłowa opcja. Wybierz 'n', 'start' lub 'exit'.");
        }
    }

    return 0;
}
