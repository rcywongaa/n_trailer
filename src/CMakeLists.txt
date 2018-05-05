set(PROJ_SRC
    main.cpp
)

find_package(drake CONFIG REQUIRED)
#find_package (Eigen3 3 REQUIRED NO_MODULE)
#find_package (Threads REQUIRED)

add_executable(car CarController.cpp main.cpp)
target_link_libraries(car drake::drake)
