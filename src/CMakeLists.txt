set(SRC
    CarController.cpp
    Tractor.cpp
    main.cpp
)

find_package(drake CONFIG REQUIRED)
#find_package (Eigen3 3 REQUIRED NO_MODULE)
#find_package (Threads REQUIRED)

add_executable(car ${SRC})
target_link_libraries(car drake::drake)
