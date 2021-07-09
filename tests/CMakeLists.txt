find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

add_rostest(node_zertz.test DEPENDENCIES ${PROJECT_NAME}_node)