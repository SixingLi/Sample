add_executable(TestAPI "")

include_directories(${SIMONE_API_ROOT}/src/cpp/APIService)
include_directories(${SIMONE_API_ROOT}/src/cpp/Sensor)
include_directories(${SIMONE_API_ROOT}/src/cpp/PNC)
include_directories(${SIMONE_API_ROOT}/src/cpp/HDMap)

include_directories(${SIMONE_PATH_ROOT}/3rdparty)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/SSD/include)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/hdmap/include)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/hdmap/include/modules)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/gtest/include)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/v2x_asn/v2x_I_New4Span/include)
include_directories(${SIMONE_PATH_ROOT}/3rdparty/tsf/include)

OPTION(WITHOUT_HDMAP "Build Source without HDMap" OFF)
IF (WITHOUT_HDMAP)
	add_definitions(-DWITHOUT_HDMAP)
ENDIF()

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-DBUILD_SIMONE_API)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

IF (MSVC)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
target_sources(TestAPI PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SetDefaultTargetProperties(TestAPI)

IF (WIN32)
	target_link_libraries(TestAPI ${CMAKE_CURRENT_LIST_DIR}/../lib/Win64/SimOneAPI.lib)
	target_link_libraries(TestAPI ${CMAKE_CURRENT_LIST_DIR}/../lib/Win64/HDMapModule.lib)
	target_link_libraries(TestAPI ${CMAKE_CURRENT_LIST_DIR}/../lib/Win64/SSD.lib)
	target_link_libraries(TestAPI ${CMAKE_CURRENT_LIST_DIR}/../lib/Win64/tsf.lib)

ELSEIF(UNIX)
	target_link_libraries(TestAPI  ${CMAKE_CURRENT_LIST_DIR}/../lib/Linux64/libSimOneAPI.so)
	target_link_libraries(TestAPI  ${CMAKE_CURRENT_LIST_DIR}/../lib/Linux64/libHDMapModule.so)
	target_link_libraries(TestAPI  ${CMAKE_CURRENT_LIST_DIR}/../lib/Linux64/libSSD.so)
	target_link_libraries(TestAPI  ${CMAKE_CURRENT_LIST_DIR}/../lib/Linux64/libtsf.so)
ENDIF()