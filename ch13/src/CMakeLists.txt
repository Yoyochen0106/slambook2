set(SRC
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        util.cpp
)

if (WIN32)
        add_library(myslam STATIC ${SRC})
else ()
        add_library(myslam SHARED ${SRC})
endif ()

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
