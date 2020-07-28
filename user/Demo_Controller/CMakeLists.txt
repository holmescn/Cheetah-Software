include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")

add_executable(demo_ctrl
    main.cpp Demo_Controller.cpp
    StateCtrl.cpp StandUpCtrl.cpp FoldLegsCtrl.cpp)
target_link_libraries(demo_ctrl robot biomimetics dynacore_param_handler qpOASES)
