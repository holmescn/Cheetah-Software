include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")
include_directories("./pybind11/include")
include_directories("/usr/local/include/eigen3")

add_subdirectory(pybind11)

add_executable(python_ctrl main.cpp BaseController.cpp PropertyHelper.cpp)
target_link_libraries(python_ctrl
    PRIVATE pybind11::embed
    robot biomimetics dynacore_param_handler qpOASES
)
set_target_properties(python_ctrl PROPERTIES COMPILE_FLAGS "-Wno-error=shadow -Wno-error=attributes")
