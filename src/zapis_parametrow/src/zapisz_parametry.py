#!/usr/bin/env python
import os
import rospy
import dynamic_reconfigure.client

def copy_parameters_to_file(node_names):
    rospy.init_node('param_copy_node')

    # Pobierz nazwę pliku od użytkownika
    custom_output_file = input("Podaj nazwę pliku do zapisu (np. params.txt): ")

    # Ustaw pełną ścieżkę do pliku w tym samym folderze, co skrypt
    script_directory = os.path.dirname(os.path.realpath(__file__))
    output_file = os.path.join(script_directory, custom_output_file)

    # Pobierz aktualne ustawienia parametrów i zapisz je do pliku
    with open(output_file, 'w') as f:
        for node_name in node_names:
            server_name = "/" + node_name
            try:
                client = dynamic_reconfigure.client.Client(server_name, timeout=30)
                current_config = client.get_configuration()
                f.write(f"{node_name.upper()}:\n")
                for key, value in current_config.items():
                    f.write(f"{key}: {value}\n")
                f.write("\n" * 5)  # Dodaj 5 nowych linii między węzłami
                rospy.loginfo(f"Zapisano parametry z węzła {node_name}")
            except rospy.ROSException as e:
                rospy.logerr(f"Błąd podczas komunikacji z węzłem {node_name}: {str(e)}")

if __name__ == '__main__':
    node_names = [
        "amcl",
        "gazebo",
        "move_base",
        "move_base/DWAPlannerROS",
        "move_base/global_costmap",
        "move_base/global_costmap/inflation_layer",
        "move_base/global_costmap/obstacle_layer",
        "move_base/global_costmap/static_layer",
        "move_base/local_costmap",
        "move_base/local_costmap/inflation_layer",
        "move_base/local_costmap/obstacle_layer",
    ]

    copy_parameters_to_file(node_names)
