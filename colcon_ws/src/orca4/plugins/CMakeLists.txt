cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(plugins)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools9 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat14 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# #============================================================================
# # Hooks
# ament_environment_hooks("hooks/resource_paths.dsv.in")
# ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================

# Plugins
list(APPEND VRX_GZ_PLUGINS
  AcousticPingerPlugin
  PublisherPlugin
)

foreach(PLUGIN ${VRX_GZ_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS ${VRX_GZ_PLUGINS}
  DESTINATION lib)


#============================================================================
# Resources
install(DIRECTORY

  DESTINATION share/${PROJECT_NAME})

ament_package()
