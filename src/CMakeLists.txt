include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
    ${PC_ORC_FRAMEWORK_INCLUDE_DIRS}
)

link_directories(
    ${PC_ORC_FRAMEWORK_LIBRARY_DIRS}
)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library 
# and installs in the directory lib/orocos/
#	
orocos_library(ModelKuka kukafixed.cpp)
orocos_component(KukaModelDemoRTNET kukaModelDemo-rtnetcomponent.cpp) # ...you may add multiple source files.
target_link_libraries(ModelKuka ${PC_ORC_FRAMEWORK_LIBRARIES})
target_link_libraries(KukaModelDemoRTNET ModelKuka)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/kuka_send_joint_positions/ directory
#
orocos_install_headers(kukaModelDemo-rtnetcomponent.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

