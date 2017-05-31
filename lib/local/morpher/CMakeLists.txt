## OPENCV
include_directories(${OpenCV_INCLUDE_DIRS})

## BASE64
include_directories(../../3rdParty/base64)
add_library(base64 ../../3rdParty/base64/base64.cpp ../../3rdParty/base64/base64.h)

## JSON
include_directories(../../3rdParty/json)

SET(SOURCE src/morph.cpp)
SET(HEADERS include/morph.h)

include_directories(${HEADERS})
add_library(morpher ${SOURCE} ${HEADERS})

install (TARGETS morpher DESTINATION lib)
