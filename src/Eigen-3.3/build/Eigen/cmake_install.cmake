# Install script for directory: /home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Cholesky"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/CholmodSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Core"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Dense"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Eigen"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Eigenvalues"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Geometry"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Householder"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/IterativeLinearSolvers"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Jacobi"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/LU"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/MetisSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/OrderingMethods"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/PaStiXSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/PardisoSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/QR"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/QtAlignedMalloc"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SPQRSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SVD"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/Sparse"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseCholesky"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseCore"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseLU"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SparseQR"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdDeque"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdList"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/StdVector"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/SuperLUSupport"
    "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/UmfPackSupport"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/khush/self_driving_car/CarND-Path-Planning-Project/src/Eigen-3.3/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

