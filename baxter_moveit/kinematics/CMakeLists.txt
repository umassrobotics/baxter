#add_executable(kinematic_model_tutorial src/kinematic_model_tutorial.cpp)
#target_link_libraries(kinematic_model_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
#target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ik_node_baxter src/ik.cpp)
target_link_libraries(ik_node_baxter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
include_directories(include)

install(TARGETS ik_node_baxter DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
