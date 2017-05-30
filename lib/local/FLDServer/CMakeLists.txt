#TBB library
include_directories(${TBB_ROOT_DIR}/include)

#BOOST library
include_directories(${BOOST_INCLUDE_DIR})

include_directories(./include)
include_directories(${FLDServer_SOURCE_DIR}/include)
include_directories(${LandmarkDetector_SOURCE_DIR}/include)
include_directories(../LandmarkDetector/include)
include_directories(../Morpher)
include_directories(../../3rdParty/base64)

SET(SOURCE
	src/io_service_pool.cpp
	src/tcp_connection.cpp
	src/tcp_server.cpp)
SET(HEADERS
	include/io_service_pool.h	
	include/tcp_connection.h
	include/tcp_server.h)

add_library(FLDServer ${SOURCE} ${HEADERS})
target_link_libraries(FLDServer base64)
target_link_libraries(FLDServer Morpher)

install(TARGETS FLDServer DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/OpenFace)
