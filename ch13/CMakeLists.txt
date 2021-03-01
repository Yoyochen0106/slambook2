cmake_minimum_required(VERSION 2.8)
project(myslam)

set(CMAKE_BUILD_TYPE Release)

if (WIN32)
    set(CMAKE_CXX_FLAGS "/bigobj /EHsc /w")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS}")
    include_directories(${EXTRA_INCLUDE_DIRS})
    set(THIRD_PARTY_LIBS ${THIRD_PARTY_LIBS} ${EXTRA_LIBS})
else ()
    set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -pthread")
    set(THIRD_PARTY_LIBS ${THIRD_PARTY_LIBS} pthread)
endif ()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
if (WIN32)
    set(G2O_LIBS g2o_core_d g2o_stuff_d g2o_types_sba_d g2o_solver_csparse_d g2o_csparse_extension_d)
else ()
    set(G2O_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)
endif ()

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

link_directories( "/usr/local/lib" )
link_directories( "${EXTRA_LINK_DIRS}" )

set(THIRD_PARTY_LIBS
        ${THIRD_PARTY_LIBS}
        ${EXTRA_LIBS}
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} # GL GLU GLEW glut
        ${G2O_LIBS}
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        ${CSPARSE_LIBRARY}
        )

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)
