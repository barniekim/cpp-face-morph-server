## OPENCV
include_directories(${OpenCV_INCLUDE_DIRS})

## BASE64
include_directories(../../3rdParty/base64)
add_library(base64 ../../3rdParty/base64/base64.cpp ../../3rdParty/base64/base64.h)

## JSON
include_directories(../../3rdParty/json)

include_directories(./)
add_library(Morpher morph.cpp)

#target_link_libraries(Morpher ${OpenCV_LIBS})
#target_link_libraries(Morpher base64)

install (TARGETS Morpher DESTINATION lib)
