INCLUDE_DIRECTORIES(/opt/etherlab/include /usr/xenomai/include ../include)
LINK_DIRECTORIES(/opt/etherlab/lib /usr/xenomai/lib)
ADD_EXECUTABLE(ecat_t1 main.cpp
global_variables.cpp
#elmo_sim.cpp
elmo_ecat.cpp
mc_power.cpp
mc_moverelative.cpp
mc_moveabsolute.cpp
jog_cp.cpp
mc_jog.cpp
robot_jog.cpp
socket_server.cpp
kine.cpp
mc_directeach.cpp)
TARGET_LINK_LIBRARIES(ecat_t1 ethercat rtdm native pthread_rt xenomai m pthread)
