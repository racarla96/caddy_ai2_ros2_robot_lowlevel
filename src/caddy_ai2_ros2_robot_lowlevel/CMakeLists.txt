# Copyright (c) 2026, racarla96 - Rafael Carbonell Lázaro
# This work is licensed under the Creative Commons Attribution 4.0 International License (CC BY 4.0).
# https://creativecommons.org/licenses/by/4.0/legalcode

cmake_minimum_required(VERSION 3.16)
project(caddy_ai2_ros2_robot_lowlevel)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
