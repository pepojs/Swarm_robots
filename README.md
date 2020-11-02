# Swarm_robots

Dodać do zmiennej środowiskowej LD_LIBRARY_PATH położenie argos3, np. :
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3

Utworzyć folder o nazwie ros_lib_links w pakiecie ros_argos3 (tam gdzie jest folder src/) oraz wewnątrz nowo utworzonego folderu stworzyć link symboliczny do biblioteki
libroscpp.so (zawartej w plikach ROS-a)

Dodać do zmiennej środowiskowej ARGOS_PLUGIN_PATH położenie ros_lib_links oraz lib od pakietu tworzącego most między ROS-e a ARGoS-em, np. :
export ARGOS_PLUGIN_PATH=/home/piotr/ARGoS/ros_argos3/src/ros_lib_links:/home/piotr/ARGoS/ros_argos3/devel/lib

Dodać do ~/.bashrc ścieszkę do pakietu ROS-a, np. 
source /home/piotr/ros_workspace/devel/setup.bash
