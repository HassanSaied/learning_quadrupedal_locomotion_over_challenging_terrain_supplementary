cmake_minimum_required(VERSION 3.10)
list(APPEND CMAKE_PREFIX_PATH $ENV{LOCAL_INSTALL})
project(blind_sandbox)

add_subdirectory(extern/pybind11)

find_package(raisim CONFIG REQUIRED)
find_package(raisimOgre 0.5.0 CONFIG REQUIRED)
find_package(TensorFlow CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

if (raisimOgre_FOUND)
    message(STATUS "Found raisimOgre!")
endif ()

if (TensorFlow_FOUND)
    message(STATUS "Found Tensorflow!")
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${TensorFlow_INCLUDE_DIRS})
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${TensorFlow_LIBRARIES})
endif ()

include_directories(include ${${PROJECT_NAME}_INCLUDE_DIRS})

add_executable(test_c100 applications/test_c100.cpp)
target_link_libraries(test_c100
        ${${PROJECT_NAME}_LIBRARIES} raisim::raisim raisim::raisimOgre Eigen3::Eigen)

add_executable(test_c010 applications/test_c010.cpp)
target_link_libraries(test_c010 ${${PROJECT_NAME}_LIBRARIES} raisim::raisim raisim::raisimOgre Eigen3::Eigen)


target_include_directories(test_c010 PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(test_c100 PUBLIC ${EIGEN3_INCLUDE_DIR})


pybind11_add_module(legged_robotics applications/pyBindSrc.cpp)
target_link_libraries(legged_robotics PUBLIC ${${PROJECT_NAME}_LIBRARIES} raisim::raisim raisim::raisimOgre Eigen3::Eigen)
target_include_directories(legged_robotics PUBLIC ${EIGEN3_INCLUDE_DIR})


# EOF
