cmake_minimum_required(VERSION 3.15...3.31)
project(csc376_franky_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The miniPC build must drive the real robot, so it sets this and fails
# without libfranka. Elsewhere only the simulated robot is built.
option(CSC376_REQUIRE_LIBFRANKA "Fail if libfranka is not found" OFF)

find_package(Threads REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
if(CSC376_REQUIRE_LIBFRANKA)
    find_package(Franka REQUIRED)
else()
    find_package(Franka QUIET)
endif()

pybind11_add_module(_control MODULE
    python/bind_franky.cpp
    src/common.cpp
    src/controller.cpp
    src/gripper_controller.cpp
    src/motion_generator.cpp
    src/sim_backend.cpp
    src/trajectory.cpp
)
target_include_directories(_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(_control PRIVATE Threads::Threads)

if(Franka_FOUND)
    # The lab's Pandas (system >= 4.2.1) need libfranka 0.9.x.
    if(NOT Franka_VERSION VERSION_GREATER_EQUAL 0.9 OR Franka_VERSION VERSION_GREATER_EQUAL 0.10)
        message(WARNING "libfranka ${Franka_VERSION} found; the lab's Panda robots need 0.9.x")
    endif()
    message(STATUS "Building the robot controller against libfranka ${Franka_VERSION}")
    target_sources(_control PRIVATE src/franka_backend.cpp)
    target_link_libraries(_control PRIVATE Franka::Franka)
    target_compile_definitions(_control PRIVATE
        CSC376_HAVE_LIBFRANKA=1
        CSC376_LIBFRANKA_VERSION="${Franka_VERSION}")
    # Find libfranka at run time even when it is not installed under /usr.
    set_target_properties(_control PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
else()
    message(WARNING "libfranka not found: building the simulated robot only")
endif()

install(TARGETS _control LIBRARY DESTINATION csc376_franky_server)
