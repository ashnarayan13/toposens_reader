find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

add_rostest_gtest(${PROJECT_NAME}-test parser_tests/parser.test parser_tests/test_parser.cpp)
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} topo_parser)

add_rostest(node_hertz.test DEPENDENCIES ${PROJECT_NAME}_node)