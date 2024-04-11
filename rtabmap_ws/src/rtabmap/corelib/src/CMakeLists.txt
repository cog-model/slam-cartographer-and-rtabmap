ADD_SUBDIRECTORY( rtabmap/proto )

SET(SRC_FILES
    CameraModel.cpp

    Transform.cpp

    util3d.cpp
    util3d_transforms.cpp

    LaserScan.cpp
    IMU.cpp

    GeodeticCoords.cpp

    StereoCameraModel.cpp

    OccupancyGridMap.cpp
    TimedOccupancyGridMap.cpp

    Time.cpp
    Color.cpp
    Trajectory.cpp
    MapLimits.cpp
    SensorData.cpp
    LocalMap.cpp
    LocalMapBuilder.cpp
    Node.cpp
    OccupancyGridBuilder.cpp
    TemporaryOccupancyGridBuilder.cpp
    DoorTracking.cpp
    ObjectTracking.cpp
    SemanticDilation.cpp
    RayTracing.cpp
    ObstacleDilation.cpp
    TrajectoriesTrimmer.cpp

    Compression.cpp
    Serialization.cpp
)

# to get includes in visual studio
IF(MSVC)
    FILE(GLOB HEADERS
        ../include/${PROJECT_PREFIX}/core/*.h
    )
    SET(SRC_FILES ${SRC_FILES} ${HEADERS})
ENDIF(MSVC)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/utilite/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${kas_utils_cpp_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIR}
)

SET(LIBRARIES
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${kas_utils_cpp_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${gflags_LIBRARIES}
)

IF(TORCH_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        ${TORCH_LIBRARIES}
    )
    SET(SRC_FILES
        ${SRC_FILES}
        superpoint_torch/SuperPoint.cc
    )
    SET(INCLUDE_DIRS
        ${TORCH_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/superpoint_torch
        ${INCLUDE_DIRS}
    )
ENDIF(TORCH_FOUND)

IF(Python3_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        Python3::Python
    )
    SET(SRC_FILES
        ${SRC_FILES}
        python/PythonInterface.cpp
        python/PyMatcher.cpp
        python/PyDetector.cpp
    )
    SET(INCLUDE_DIRS
        ${TORCH_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/python
        ${INCLUDE_DIRS}
    )
ENDIF(Python3_FOUND)

IF(Freenect_FOUND)
    IF(Freenect_DASH_INCLUDES)
        ADD_DEFINITIONS("-DFREENECT_DASH_INCLUDES")
    ENDIF(Freenect_DASH_INCLUDES)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${Freenect_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${Freenect_LIBRARIES}
    )
ENDIF(Freenect_FOUND)

IF(OpenNI2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${OpenNI2_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${OpenNI2_LIBRARIES}
    )
ENDIF(OpenNI2_FOUND)

IF(freenect2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${freenect2_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${freenect2_LIBRARIES}
    )
ENDIF(freenect2_FOUND)

IF(KinectSDK2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${KinectSDK2_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${KinectSDK2_LIBRARIES}
    )
ENDIF(KinectSDK2_FOUND)

IF(k4a_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${k4a_INCLUDE_DIRS}
    )
    IF(WIN32)
        SET(LIBRARIES
            ${LIBRARIES}
            ${k4a_LIBRARIES}
        )
    ELSE()
        SET(LIBRARIES
            ${LIBRARIES}
            k4a::k4a 
            k4a::k4arecord
        )
    ENDIF()
ENDIF(k4a_FOUND)

IF(RealSense_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${RealSense_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${RealSense_LIBRARIES}
    )
ENDIF(RealSense_FOUND)

IF(realsense2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${realsense2_INCLUDE_DIRS}
    )
    IF(WIN32)
        SET(LIBRARIES
            ${LIBRARIES}
            ${RealSense2_LIBRARIES}
        )
    ELSEIF(APPLE)
        SET(LIBRARIES
            ${LIBRARIES}
            ${realsense2_LIBRARIES}
        )
    ELSE()
        SET(LIBRARIES
            ${LIBRARIES}
            realsense2::realsense2 # target
        )
    ENDIF()
ENDIF(realsense2_FOUND)

IF(DC1394_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${DC1394_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${DC1394_LIBRARIES}
    )
ENDIF(DC1394_FOUND)

IF(FlyCapture2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${FlyCapture2_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${FlyCapture2_LIBRARIES}
    )
ENDIF(FlyCapture2_FOUND)

IF(mynteye_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        mynteye # target
    )
ENDIF(mynteye_FOUND)

IF(depthai_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        depthai::core
        depthai::opencv
    )
ENDIF(depthai_FOUND)

IF(TARGET OpenMP::OpenMP_CXX)
    SET(LIBRARIES
        ${LIBRARIES}
        OpenMP::OpenMP_CXX
    )
ENDIF(TARGET OpenMP::OpenMP_CXX)

IF(WITH_TORO)
    SET(SRC_FILES
        ${SRC_FILES}
        optimizer/toro3d/posegraph3.cpp
        optimizer/toro3d/treeoptimizer3_iteration.cpp
        optimizer/toro3d/treeoptimizer3.cpp
    
        optimizer/toro3d/posegraph2.cpp
        optimizer/toro3d/treeoptimizer2.cpp
    )
ENDIF(WITH_TORO)

IF(G2O_FOUND)
    IF(g2o_FOUND)
      SET(LIBRARIES
             ${LIBRARIES}
         g2o::core
         g2o::solver_cholmod
         g2o::solver_eigen
         g2o::solver_pcg
         g2o::solver_csparse
         g2o::csparse_extension
         g2o::types_slam2d
         g2o::types_slam3d
         g2o::types_sba
      )
    ELSE()
      SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${G2O_INCLUDE_DIRS}
      )
      SET(LIBRARIES
        ${LIBRARIES}
        ${G2O_LIBRARIES}
      )
    ENDIF()
    SET(SRC_FILES
            ${SRC_FILES}
        optimizer/g2o/edge_se3_xyzprior.cpp
    )
    IF(WITH_VERTIGO)
        SET(SRC_FILES
                ${SRC_FILES}
            optimizer/vertigo/g2o/edge_se2Switchable.cpp
            optimizer/vertigo/g2o/edge_se3Switchable.cpp
            optimizer/vertigo/g2o/edge_switchPrior.cpp
            optimizer/vertigo/g2o/types_g2o_robust.cpp
            optimizer/vertigo/g2o/vertex_switchLinear.cpp
        )
    ENDIF(WITH_VERTIGO)
ENDIF(G2O_FOUND)

IF(cvsba_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${cvsba_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${cvsba_LIBS}
    )
ENDIF(cvsba_FOUND)

IF(CERES_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${CERES_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${CERES_LIBRARIES}
    )
ENDIF(CERES_FOUND)

IF(libpointmatcher_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${libpointmatcher_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${libpointmatcher_LIBRARIES}
    )
ENDIF(libpointmatcher_FOUND)

IF(CCCoreLib_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        CCCoreLib::CCCoreLib
    )
ENDIF(CCCoreLib_FOUND)

IF(FastCV_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${FastCV_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${FastCV_LIBRARIES}
    )
ENDIF(FastCV_FOUND)

IF(PDAL_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${PDAL_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${PDAL_LIBRARIES}
    )
    SET(SRC_FILES
        ${SRC_FILES}
        PDALWriter.cpp
    )
    IF(PDAL_VERSION VERSION_LESS "1.7")
        add_definitions("-DRTABMAP_PDAL_16")
    ENDIF(PDAL_VERSION VERSION_LESS "1.7")
ENDIF(PDAL_FOUND)

IF(loam_velodyne_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${loam_velodyne_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${loam_velodyne_LIBRARIES}
    )
ENDIF(loam_velodyne_FOUND)

IF(ZED_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${ZED_LIBRARIES}
    )
    IF(CUDA_FOUND)
        SET(INCLUDE_DIRS
            ${INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
        )
        SET(LIBRARIES
            ${LIBRARIES}
            ${CUDA_LIBRARIES}
        )
    ENDIF(CUDA_FOUND)
ENDIF(ZED_FOUND)

IF(ZEDOC_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${ZEDOC_INCLUDE_DIRS}
        ${HIDAPI_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${ZEDOC_LIBRARIES}
        ${HIDAPI_LIBRARIES}
    )
ENDIF(ZEDOC_FOUND)

IF(octomap_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${OCTOMAP_LIBRARIES}
    )
    SET(SRC_FILES
        ${SRC_FILES}
        OctoMap.cpp
    )
ENDIF(octomap_FOUND)

IF(AliceVision_FOUND)
  SET(LIBRARIES
    ${LIBRARIES}
    aliceVision_mesh 
    aliceVision_sfmDataIO)
ENDIF(AliceVision_FOUND)

IF(libfovis_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${libfovis_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${libfovis_LIBRARIES}
    )
ENDIF(libfovis_FOUND)

IF(libviso2_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${libviso2_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${libviso2_LIBRARIES}
    )
ENDIF(libviso2_FOUND)

IF(dvo_core_FOUND)
    SET(INCLUDE_DIRS
        ${INCLUDE_DIRS}
        ${dvo_core_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${dvo_core_LIBRARIES}
    )
ENDIF(dvo_core_FOUND)

IF(okvis_FOUND)
    SET(INCLUDE_DIRS
        ${OKVIS_INCLUDE_DIRS}
        ${BRISK_INCLUDE_DIRS}
        ${OPENGV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${OKVIS_LIBRARIES}
        ${BRISK_LIBRARIES}
        ${OPENGV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${LIBRARIES}
    )
ENDIF(okvis_FOUND)

IF(msckf_vio_FOUND)
    SET(INCLUDE_DIRS
        ${msckf_vio_INCLUDE_DIRS}
        ${INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${msckf_vio_LIBRARIES}
        ${LIBRARIES}
    )
ENDIF(msckf_vio_FOUND)

IF(vins_FOUND)
    SET(INCLUDE_DIRS
        ${vins_INCLUDE_DIRS}
        ${INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${vins_LIBRARIES}
        ${LIBRARIES}
    )
ENDIF(vins_FOUND)

IF(ov_msckf_FOUND)
    SET(INCLUDE_DIRS
        ${ov_msckf_INCLUDE_DIRS}
        ${INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${ov_msckf_LIBRARIES}
        ${LIBRARIES}
    )
ENDIF(ov_msckf_FOUND)

IF(ORB_SLAM_FOUND)
    SET(INCLUDE_DIRS
        ${ORB_SLAM_INCLUDE_DIRS} #before so that g2o includes are taken from ORB_SLAM directory before the official g2o one
        ${INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${ORB_SLAM_LIBRARIES}
        ${LIBRARIES}
    )
ENDIF(ORB_SLAM_FOUND)

IF(GTSAM_FOUND)
    # Make sure GTSAM is built with system Eigen, not the included one in its package
    IF(GTSAM_INCLUDE_DIR)
        SET(INCLUDE_DIRS 
            ${INCLUDE_DIRS} 
            ${GTSAM_INCLUDE_DIR} 
        )
    ELSE()
        SET(INCLUDE_DIRS 
            ${INCLUDE_DIRS} 
            ${GTSAM_INCLUDE_DIRS}
        )
    ENDIF()
    SET(SRC_FILES
        ${SRC_FILES}
        optimizer/gtsam/GravityFactor.cpp
    )
    IF(WIN32)
        # GTSAM should be built in STATIC on Windows to avoid "error C2338: THIS_METHOD_IS_ONLY_FOR_1x1_EXPRESSIONS" when building GTSAM
        add_definitions("-DGTSAM_IMPORT_STATIC") 
    ENDIF(WIN32)
    SET(LIBRARIES
        ${LIBRARIES}
        gtsam # Windows: Place static libs at the end
    )
ENDIF(GTSAM_FOUND)

####################################
# Generate resources files
####################################
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql.in ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql)

SET(RESOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_18_3.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_18_0.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_17_0.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_2.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_1.sql
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_0.sql
)

foreach(arg ${RESOURCES})
    get_filename_component(filename ${arg} NAME)
    string(REPLACE "." "_" output ${filename})
    set(RESOURCES_HEADERS "${RESOURCES_HEADERS}" "${CMAKE_CURRENT_BINARY_DIR}/${output}.h")
endforeach(arg ${RESOURCES})

#MESSAGE(STATUS "RESOURCES = ${RESOURCES}")
#MESSAGE(STATUS "RESOURCES_HEADERS = ${RESOURCES_HEADERS}")

IF(ANDROID)
    IF(NOT RTABMAP_RES_TOOL)
      find_host_program(RTABMAP_RES_TOOL rtabmap-res_tool PATHS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
      IF(NOT RTABMAP_RES_TOOL)
         MESSAGE( FATAL_ERROR "RTABMAP_RES_TOOL is not defined (it is the path to \"rtabmap-res_tool\" application created by a non-Android build)." )
      ENDIF(NOT RTABMAP_RES_TOOL)
    ENDIF(NOT RTABMAP_RES_TOOL)

    ADD_CUSTOM_COMMAND(
      OUTPUT ${RESOURCES_HEADERS}
      COMMAND ${RTABMAP_RES_TOOL} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
      COMMENT "[Creating resources]"
      DEPENDS ${RESOURCES}
    )
ELSE()
    ADD_CUSTOM_COMMAND(
      OUTPUT ${RESOURCES_HEADERS}
      COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rtabmap-res_tool -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
      COMMENT "[Creating resources]"
      DEPENDS ${RESOURCES} res_tool
    )
ENDIF()

####################################
# Generate resources files END
####################################

ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
SET_SOURCE_FILES_PROPERTIES(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)
ADD_LIBRARY(rtabmap_core ${SRC_FILES} ${PROTO_SRCS} ${RESOURCES_HEADERS})
ADD_DEPENDENCIES(rtabmap_core generated_protobuf_code)
TARGET_LINK_LIBRARIES(rtabmap_core rtabmap_utilite ${LIBRARIES})

ADD_EXECUTABLE(ocp_map_to_image ocp_map_to_image.cpp)
TARGET_LINK_LIBRARIES(ocp_map_to_image rtabmap_core)

SET_TARGET_PROPERTIES(
     rtabmap_core
    PROPERTIES
     VERSION ${RTABMAP_VERSION}
     SOVERSION ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}
)

INSTALL(TARGETS rtabmap_core
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
        
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ 
        DESTINATION "${INSTALL_INCLUDE_DIR}"
        COMPONENT devel 
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" 
        PATTERN ".svn" EXCLUDE)

install(FILES ${PROTO_HDRS}
        DESTINATION "${INSTALL_INCLUDE_DIR}/rtabmap/proto"
        COMPONENT devel)

