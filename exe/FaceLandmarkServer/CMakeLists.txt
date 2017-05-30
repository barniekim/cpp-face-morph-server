#TBB library
include_directories(${TBB_ROOT_DIR}/include)

# Local libraries
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
include_directories(${FLDServer_SOURCE_DIR}/include)
include_directories(../../lib/local/LandmarkDetector/include)
include_directories(../../lib/local/FLDServer/include)
include_directories(../../lib/local/Morpher)

add_executable(FaceLandmarkServer FaceLandmarkServer.cpp)

target_link_libraries(FaceLandmarkServer FLDServer)
target_link_libraries(FaceLandmarkServer LandmarkDetector)
target_link_libraries(FaceLandmarkServer dlib)
target_link_libraries(FaceLandmarkServer Morpher)
target_link_libraries(FaceLandmarkServer ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

install (TARGETS FaceLandmarkServer DESTINATION bin)
