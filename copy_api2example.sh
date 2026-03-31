#!/bin/bash

API_ROOT_DIR=${PWD}/_archive_/
ROS1_EXAMPLE_DIR=${PWD}/examples/ros_ml/src/ml
ROS2_EXAMPLE_DIR=${PWD}/examples/ros2_ml/src/ml

rm -rf ${ROS1_EXAMPLE_DIR}/libsoslab/*
rm -rf ${ROS1_EXAMPLE_DIR}/include/*
rm -rf ${ROS2_EXAMPLE_DIR}/libsoslab/*
rm -rf ${ROS2_EXAMPLE_DIR}/include/*

mkdir -p ${ROS1_EXAMPLE_DIR}/libsoslab
mkdir -p ${ROS1_EXAMPLE_DIR}/include
mkdir -p ${ROS2_EXAMPLE_DIR}/libsoslab
mkdir -p ${ROS2_EXAMPLE_DIR}/include

cp -r ${API_ROOT_DIR}/lib/*.so ${ROS1_EXAMPLE_DIR}/libsoslab
cp -r ${API_ROOT_DIR}/include/* ${ROS1_EXAMPLE_DIR}/include
cp -r ${API_ROOT_DIR}/lib/*.so ${ROS2_EXAMPLE_DIR}/libsoslab
cp -r ${API_ROOT_DIR}/include/* ${ROS2_EXAMPLE_DIR}/include
