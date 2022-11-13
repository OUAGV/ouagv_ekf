cmake_minimum_required(VERSION 3.8)
project(ouagv_ekf)
set(COMP_NAME "ouagv_ekf")
string(TOLOWER ${COMP_NAME} COMP_NAME_LOWER)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    # set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Create the component target
ament_auto_add_library(${COMP_NAME_LOWER}_component SHARED
    src/${COMP_NAME_LOWER}_component.cpp
)
target_compile_definitions(${COMP_NAME_LOWER}_component
    PRIVATE ${PROJECT_NAME}_BUILDING_DLL)
rclcpp_components_register_nodes(${COMP_NAME_LOWER}_component
    "${PROJECT_NAME}::EkfComponent")

# Create the executor target
ament_auto_add_executable(${COMP_NAME_LOWER}_node
    NO_TARGET_LINK_LIBRARIES
    src/${COMP_NAME_LOWER}_node.cpp
)
target_link_libraries(${COMP_NAME_LOWER}_node
    ${COMP_NAME_LOWER}_component Eigen3::Eigen)

ament_auto_package()
