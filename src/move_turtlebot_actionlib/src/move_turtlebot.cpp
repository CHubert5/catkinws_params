#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <iostream>
#include <limits>
#include <tf/transform_datatypes.h>
#include <termios.h>
#include <sys/ioctl.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static bool isGoalReached = true; // Globalna flaga informująca, czy osiągnięto cel

void goalDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Osiagnieto cel");
        isGoalReached = true;
    } else {
        ROS_INFO("Nie udalo sie osiagnac celu");
        isGoalReached = false;
    }
}

bool kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

class waypoint{
private:
    double pos_x;
    double pos_y;
    double orientation_rad;
    waypoint * next;
public:
    void init();
    void clear();
    void fill();
    void add_waypoint();
    void send_waypoint(MoveBaseClient& ac);
    waypoint * next_waypoint();
    void list();
};

void waypoint::init(){
    next = NULL;
}

void waypoint::clear(){
    if(next != NULL){
        next->clear();
        delete next;
        next = NULL;
    }
}

waypoint * waypoint::next_waypoint(){
    return(next);
}

void waypoint::add_waypoint(){
    waypoint * last_waypoint = this;
    while(last_waypoint->next != NULL){
        last_waypoint = last_waypoint->next;
    }

    waypoint * new_waypoint = new waypoint;
    new_waypoint->init();
    new_waypoint->fill();
    last_waypoint->next = new_waypoint;
}

void waypoint::fill(){
    double orientation_deg;

    std::cout << "Podaj pozycje x: ";
    std::cin >> pos_x;

    std::cout << "Podaj pozycje y: ";
    std::cin >> pos_y;

    std::cout << "Podaj orientacje w stopniach (-180 do 180): ";
    std::cin >> orientation_deg;

    orientation_rad = orientation_deg * M_PI / 180.0;
    // Upewnij się, że orientacja jest w zakresie od -180 do 180 stopni
    if (orientation_deg < -180.0 || orientation_deg > 180.0) {
        ROS_ERROR("Nieprawidlowa orientacja. Orientacja powinna byc w zakresie od -180 do 180 stopni.");
    }
}

void waypoint::send_waypoint(MoveBaseClient& ac){
    move_base_msgs::MoveBaseGoal goal;

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Oczekiwanie na serwer akcji move_base");
    }
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pos_x;
    goal.target_pose.pose.position.y = pos_y;

    // Ustaw orientację w wiadomości
    tf::Quaternion quat;
    quat.setRPY(0, 0, orientation_rad); // Roll, Pitch, Yaw
    tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);

    ROS_INFO("Wysylanie celu");
    ac.sendGoal(goal, &goalDoneCallback);
    isGoalReached = false;
}

void waypoint::list(){
    std::cout << "Pozycja x: " << pos_x << '\n' << "Pozycja y: " << pos_y << '\n' << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_turtlebot");

    MoveBaseClient ac("move_base", true);

    std::string input;

    waypoint * first_waypoint = new waypoint;
    first_waypoint->init();
    first_waypoint->fill();

    while (ros::ok()){
        if (kbhit() == true){
            std::cout << "W celu uruchomienia symulacji nalezy wpisac 'Start', do tego czasu kolejne wspolrzedne moga byc wpisywane przez komende 'n'." << std::endl;
            std::cin >> input;
            if (input == "n"){
                first_waypoint->add_waypoint();
            } else if (input == "Start"){
                std::cout << "Symulacja uruchomiona." << std::endl;
                break;
            }
        }
    }

    waypoint * current = first_waypoint;
    first_waypoint->send_waypoint(ac);

    while (ros::ok()){
        if (kbhit() == true){
            std::cout << "Aby przerwac akcje, wpisz 'q' i nacisnij Enter. \nAby ustawic nowy cel, wpisz 'n' i nacisnij Enter. \nAby wypisac liste wspolrzednych, wpisz 'list' i nacisnij Enter. \nAby zresetowac cel wpisz 'reset' i nacisnij Enter. \nAby zobaczyc aktualny cel wpisz 'current' i nacisnij Enter. \nAby zatrzymac robota wpisz 'abort' i nacisnij Enter. \nAby zakonczyc program, wpisz 'exit' i nacisnij Enter: \n \n";
            std::cin >> input;
            if (input == "q") {
                ROS_INFO("Przerwano akcje na zadanie uzytkownika.");
                isGoalReached = 1;
                ac.cancelGoal();
            } else if (input == "n") {
                ROS_INFO("Podaj nowy cel.");
                first_waypoint->add_waypoint();
            } else if (input == "exit") {
                ROS_INFO("Zakonczono program.");
                first_waypoint->clear();
                return 0;
            } else if (input == "list"){
                waypoint * list = first_waypoint;
                std::cout << "Lista celow od poczatku dzialania programu: " << std::endl;
                while(list != NULL){
                    list->list();
                    std::cout << std::endl;
                    list = list->next_waypoint();
                }
            } else if (input == "current"){
                std::cout << "AKtualne wspolrzedne celu: " << std::endl;
                current->list();
            } else if (input == "stan"){
                std::cout << "Aktualny stan celu: " << isGoalReached << std::endl;
            } else if (input == "reset"){
                isGoalReached = 1;
                ac.cancelGoal();
                current->send_waypoint(ac);
            } else if (input == "abort"){
                isGoalReached = 1;
                ac.cancelGoal();
                first_waypoint->clear();
            }

            ros::spinOnce();
        }
        if(isGoalReached && current->next_waypoint() != NULL){
            current = current->next_waypoint();
            current->send_waypoint(ac);
        }
    }
    return 0;
}
