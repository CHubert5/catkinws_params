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

bool kbhit() {
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

class Coordinates {
private:
    double pos_x;
    double pos_y;
    double orientation_rad;
    Coordinates* next;

public:
    void init();
    void clean();
    void fill();
    void add_coordinates();
    void send_coordinates(MoveBaseClient& ac);
    Coordinates* next_coordinates();
    void listed();
};

void Coordinates::init() {
    next = NULL;
}

void Coordinates::clean() {
    if (next != NULL) {
        next->clean();
        delete next;
        next = NULL;
    }
}

Coordinates* Coordinates::next_coordinates() {
    return (next);
}

void Coordinates::add_coordinates() {
    Coordinates* last_coordinates = this;
    while (last_coordinates->next != NULL) {
        last_coordinates = last_coordinates->next;
    }

    Coordinates* new_coordinates = new Coordinates;
    new_coordinates->init();
    new_coordinates->fill();
    last_coordinates->next = new_coordinates;
}

void Coordinates::fill() {
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

void Coordinates::send_coordinates(MoveBaseClient& ac) {
    move_base_msgs::MoveBaseGoal goal;

    while (!ac.waitForServer(ros::Duration(5.0))) {
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

void Coordinates::listed() {
    std::cout << "Pozycja x: " << pos_x << '\n' << "Pozycja y: " << pos_y << '\n' << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_turtlebot");

    MoveBaseClient ac("move_base", true);

    Coordinates* first_coordinates = new Coordinates;
    first_coordinates->init();
    first_coordinates->fill();

    Coordinates* current = first_coordinates;
    first_coordinates->send_coordinates(ac);

    std::string input;
    bool showMenu = true;

    while (ros::ok()) {
        if (showMenu) {
            std::cout << "Wybierz akcję:" << std::endl;
            std::cout << "1. Dodaj new współrzędne" << std::endl;
            std::cout << "2. Przerwij akcję" << std::endl;
            std::cout << "3. Wyświetl listę współrzędnych" << std::endl;
            std::cout << "4. Wyświetl aktualne współrzędne celu" << std::endl;
            std::cout << "5. Zakończ program" << std::endl;
            showMenu = false;
        }

        if (kbhit() == true) {
            std::cin >> input;
            
            if (input == "1") {
                ROS_INFO("Podaj new współrzędne.");
                first_coordinates->add_coordinates();
            } else if (input == "2") {
                ROS_INFO("Przerwano akcję na żądanie użytkownika.");
                ac.cancelGoal();
            } else if (input == "3") {
                Coordinates* lista = first_coordinates;
                std::cout << "Lista celów od początku działania programu: " << std::endl;
                while (lista != NULL) {
                    lista->listed();
                    std::cout << std::endl;
                    lista = lista->next_coordinates();
                }
            } else if (input == "4") {
                std::cout << "Aktualne współrzędne celu: " << std::endl;
                current->listed();
            } else if (input == "5") {
                ROS_INFO("Zakończono program.");
                if (first_coordinates != nullptr) {
                    first_coordinates->clean();
                }
                return 0;
            } else {
                ROS_WARN("Nieprawidłowy wybór. Wybierz akcję od 1 do 5.");
            }

            showMenu = true;
        }}}

