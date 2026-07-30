cmake_minimum_required(VERSION 3.8)

project(ninezzhou)

find_package(ament_cmake REQUIRED)

foreach(directory config launch meshes urdf)
  install(
    DIRECTORY ${directory}/
    DESTINATION share/${PROJECT_NAME}/${directory}
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE
  )
endforeach()

ament_package()
