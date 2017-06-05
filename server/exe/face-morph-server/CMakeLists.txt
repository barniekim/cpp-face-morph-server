#TBB library
include_directories(${TBB_ROOT_DIR}/include)

# Local libraries
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
include_directories(${fld-server_SOURCE_DIR}/include)
include_directories(../../lib/local/LandmarkDetector/include)
include_directories(../../lib/local/fld-server/include)
include_directories(../../lib/local/morpher/include)

add_executable(face-morph-server face-morph-server.cpp)

target_link_libraries(face-morph-server fld-server morpher dlib LandmarkDetector ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

install (TARGETS face-morph-server DESTINATION bin)
