
# set(CMAKE_C_COMPILER "/usr/local/gcc")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++")

add_executable(LimitChangeLane "")

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp *.py)
file(GLOB_RECURSE headers *.h)

target_sources(LimitChangeLane PRIVATE "src/LimitChangeLaneSample.cpp" ${headers})

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

# include_directories("${SIMONE_API}/include/")
include_directories("${CMAKE_CURRENT_LIST_DIR}/../util/")
include_directories("${INCLUDE_DIR}")
include_directories(${CMAKE_CURRENT_LIST_DIR}/eigen/3.2.3/include/eigen3)
#include_directories("${CMAKE_CURRENT_LIST_DIR}/../../Workshop/util")
#include_directories("${3RDPARTY}")

# find_package(OpenCV REQUIRED)
# target_link_libraries(image ${OpenCV_LIBS})

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()
	add_definitions(-DNOMINMAX)
	add_definitions(-DUNICODE)
	add_definitions(-D_UNICODE)

IF (MSVC)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

SetDefaultTargetProperties(LimitChangeLane)


IF (WIN32)
		target_link_libraries(LimitChangeLane  ${LIB_DIR}/SimOneAPI.lib ${LIB_DIR}/SSD.lib ${LIB_DIR}/HDMapModule.lib  ${LIB_DIR}/ProtobufModule.lib)
 

    add_custom_command(TARGET LimitChangeLane
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SimOneAPI.dll ${BINARY_OUT_PATH}/SimOneAPI.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SSD.dll ${BINARY_OUT_PATH}/SSD.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/HDMapModule.dll ${BINARY_OUT_PATH}/HDMapModule.dll
        )
ELSEIF (UNIX)
        # target_link_libraries(image_cv debug ${OpenCV_LIBS})
        # target_link_libraries(image_cv optimized ${OpenCV_LIBS})
	
	target_link_libraries(LimitChangeLane debug ${LIB_DIR}/libSimOneAPI.so)
        target_link_libraries(LimitChangeLane optimized ${LIB_DIR}/libSimOneAPI.so -lpthread)
        target_link_libraries(LimitChangeLane debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        target_link_libraries(LimitChangeLane optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        target_link_libraries(LimitChangeLane debug ${LIB_DIR}/libSSD.so)
        target_link_libraries(LimitChangeLane optimized ${LIB_DIR}/libSSD.so -lpthread)
        target_link_libraries(LimitChangeLane debug ${LIB_DIR}/libHDMapModule.so)
        target_link_libraries(LimitChangeLane optimized ${LIB_DIR}/libHDMapModule.so)
    
        add_custom_command(TARGET LimitChangeLane
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneAPI.so ${BINARY_OUT_PATH}/libSimOneAPI.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so
        )
ENDIF()
