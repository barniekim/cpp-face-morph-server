#TBB library
include_directories(${TBB_ROOT_DIR}/include)

#BOOST library
include_directories(${BOOST_INCLUDE_DIR})

include_directories(../fld-core/include)
include_directories(../morpher/include)
include_directories(${fld-server_SOURCE_DIR}/include)
include_directories(${fld-core_SOURCE_DIR}/include)
include_directories(./include)
include_directories(../../3rdParty/base64)

SET(SOURCE
	src/io_service_pool.cpp
	src/tcp_connection.cpp
	src/tcp_server.cpp)
SET(HEADERS
	include/io_service_pool.h	
	include/tcp_connection.h
	include/tcp_server.h)

add_library(fld-server ${SOURCE} ${HEADERS})
target_link_libraries(fld-server base64)
target_link_libraries(fld-server morpher)

install(TARGETS fld-server DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/OpenFace)
