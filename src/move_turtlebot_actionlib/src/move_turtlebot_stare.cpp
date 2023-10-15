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

class Wspolrzedne{
private:
    double pos_x;
    double pos_y;
    double orientation_rad;
    Wspolrzedne * next;
public:
    void inicjuj();
    void sprzatnij();
    void wypelnij();
    void dodaj_wspolrzedne();
    void wyslij_wspolrzedne(MoveBaseClient& ac);
    Wspolrzedne * nastepne_wspolrzedne();
    void wypisz();
};

void Wspolrzedne::inicjuj(){
    next = NULL;
}

void Wspolrzedne::sprzatnij(){
    if(next != NULL){
        next->sprzatnij();
        delete next;
        next = NULL;
    }
}

Wspolrzedne * Wspolrzedne::nastepne_wspolrzedne(){
    return(next);
}

void Wspolrzedne::dodaj_wspolrzedne(){
    Wspolrzedne * ostatnie_wspolrzedne = this;
    while(ostatnie_wspolrzedne->next != NULL){
        ostatnie_wspolrzedne = ostatnie_wspolrzedne->next;
    }

    Wspolrzedne * nowe_wspolrzedne = new Wspolrzedne;
    nowe_wspolrzedne->inicjuj();
    nowe_wspolrzedne->wypelnij();
    ostatnie_wspolrzedne->next = nowe_wspolrzedne;
}

void Wspolrzedne::wypelnij(){
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

void Wspolrzedne::wyslij_wspolrzedne(MoveBaseClient& ac){
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

void Wspolrzedne::wypisz(){
    std::cout << "Pozycja x: " << pos_x << '\n' << "Pozycja y: " << pos_y << '\n' << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_turtlebot");

    MoveBaseClient ac("move_base", true);

    Wspolrzedne * pierwsze_wspolrzedne = new Wspolrzedne;
    pierwsze_wspolrzedne->inicjuj();
    pierwsze_wspolrzedne->wypelnij();

    Wspolrzedne * aktualne = pierwsze_wspolrzedne;
    pierwsze_wspolrzedne->wyslij_wspolrzedne(ac);

    std::string input;

    while (ros::ok()){
        if (kbhit() == true){
            std::cout << "Aby przerwac akcje, wpisz 'q' i nacisnij Enter. Aby ustawic nowy cel, wpisz 'n' i nacisnij Enter. Aby wypisać listę wspolrzednych, wpisz 'lista' i nacisnij Enter. Aby zobaczyc aktualny cel wpisz 'aktualne' i nacisnij Enter. Aby zakonczyc program, wpisz 'exit' i nacisnij Enter: ";
            std::cin >> input;
            if (input == "q") {
                ROS_INFO("Przerwano akcje na zadanie uzytkownika.");
                ac.cancelGoal();
            } else if (input == "n") {
                ROS_INFO("Podaj nowy cel.");
                pierwsze_wspolrzedne->dodaj_wspolrzedne();
            } else if (input == "exit") {
                ROS_INFO("Zakonczono program.");
                pierwsze_wspolrzedne->sprzatnij();
                return 0;
            } else if (input == "lista"){
                Wspolrzedne * lista = pierwsze_wspolrzedne;
                std::cout << "Lista celow od poczatku dzialania programu: " << std::endl;
                while(lista != NULL){
                    lista->wypisz();
                    std::cout << std::endl;
                    lista = lista->nastepne_wspolrzedne();
                }
            } else if (input == "aktualne"){
                std::cout << "Aktualne wspolrzedne celu: " << std::endl;
                aktualne->wypisz();
            } else if (input == "stan"){
                std::cout << "Aktualny stan celu: " << isGoalReached << std::endl;
            }

            ros::spinOnce();
        }
        if(isGoalReached && aktualne->nastepne_wspolrzedne() != NULL){
            aktualne = aktualne->nastepne_wspolrzedne();
            aktualne->wyslij_wspolrzedne(ac);
        }
    }
    return 0;
}
