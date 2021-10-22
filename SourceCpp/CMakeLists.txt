cmake_minimum_required(VERSION 3.8.0)

add_library(${CMAKE_BUILD_NAME} SHARED "")

OPTION(HDMAP "Build ${CMAKE_BUILD_NAME}  HDMap" ON)
IF (HDMAP)
	add_definitions(-DHDMAP)
ENDIF()

OPTION(SENSOR "Build ${CMAKE_BUILD_NAME}  Sensor" ON)
IF (SENSOR)
	add_definitions(-DSENSOR)
ENDIF()

OPTION(PNC "Build ${CMAKE_BUILD_NAME}  Planning and Control" ON)
IF (PNC)
	add_definitions(-DPNC)
ENDIF()

IF (HDMAP)
	include_directories("${SIMONE_PATH_ROOT}/3rdparty/SSD/include")
	include_directories("${SIMONE_PATH_ROOT}/3rdparty/hdmap/include")
	include_directories("${SIMONE_PATH_ROOT}/3rdparty/hdmap/include/modules")
	include_directories("${SIMONE_PATH_ROOT}/3rdparty/SSD/include")
ENDIF()

IF (SENSOR)
	include_directories("${SIMONE_PATH_ROOT}/3rdparty/artifactory/ffmpeg/include")
ENDIF()

include_directories("${SIMONE_PATH_FOUNDATION}/Source/CoreLibrary/include")
include_directories("${SIMONE_PATH_FOUNDATION}/Source/Protocol/Output/cpp")
include_directories("${SIMONE_PATH_FOUNDATION}/Source/Worker/BaseLibrary/NodeBase")
include_directories("${SIMONE_PATH_FOUNDATION}/Source/Worker/BaseLibrary/SensorBase/include")
include_directories("${SIMONE_PATH_FOUNDATION}/Source/Worker/BaseLibrary/BridgeBase/include")
include_directories("${SIMONE_PATH_FOUNDATION}/Source/Logger/LogApi")

include_directories("${SIMONE_API_ROOT}/SourceCpp/APIService")
include_directories("${SIMONE_API_ROOT}/SourceCpp/HDMap")
include_directories("${SIMONE_API_ROOT}/SourceCpp/Sensor")

IF (WIN32)
	add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()

include_directories("${SIMONE_PATH_ROOT}/3rdparty/asio")
include_directories("${SIMONE_PATH_ROOT}/3rdparty")

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)
add_definitions(-DCURL_STATICLIB)
add_definitions(-DBUILD_SIMONE_API)
add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)
IF (MSVC)
	string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

IF (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
ENDIF()

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp *.py)

target_sources(${CMAKE_BUILD_NAME} PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SetDefaultTargetProperties(${CMAKE_BUILD_NAME})
SET(FFMPEG_LIB_PATH ${SIMONE_PATH_ROOT}/3rdparty/artifactory/ffmpeg/lib)

IF (WIN32)
	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Debug)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Release)

	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/protobuf.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/CybertronCore.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/CybertronBridgeBase.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/CybertronSensorBase.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/CybertronLogApi.lib)

	IF (HDMAP)
		target_link_libraries(${CMAKE_BUILD_NAME} debug ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/win64/debug/SSD.lib)
		target_link_libraries(${CMAKE_BUILD_NAME} debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/debug/HDMapModule.lib)
		target_link_libraries(${CMAKE_BUILD_NAME} optimized ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/win64/release/SSD.lib)
		target_link_libraries(${CMAKE_BUILD_NAME} optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/release/HDMapModule.lib)
	ENDIF()

	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/protobuf.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/CybertronCore.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/CybertronBridgeBase.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/CybertronSensorBase.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/CybertronLogApi.lib)
	target_link_libraries(${CMAKE_BUILD_NAME} ws2_32)

ELSEIF (UNIX)
	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_debug/lib)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_release/lib)
	set_target_properties(${CMAKE_BUILD_NAME} PROPERTIES LINK_FLAGS "-Wl,--exclude-libs,ALL")

	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/libCybertronCore.a  -pthread -lrt)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/libprotobuf.a)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/libCybertronBridgeBase.a)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/libCybertronSensorBase.a)
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${FOUNDATION_DBG_LIB_PATH}/libCybertronLogApi.a)

	IF (HDMAP)
		target_link_libraries(${CMAKE_BUILD_NAME} debug ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/linux/debug/libSSD.so)
		target_link_libraries(${CMAKE_BUILD_NAME} debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/debug/libHDMapModule.so)
		target_link_libraries(${CMAKE_BUILD_NAME} optimized ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/linux/release/libSSD.so)
		target_link_libraries(${CMAKE_BUILD_NAME} optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/release/libHDMapModule.so)
	ENDIF()

	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/libCybertronCore.a  -pthread -lrt)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/libprotobuf.a)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/libCybertronBridgeBase.a)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/libCybertronSensorBase.a)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${FOUNDATION_REL_LIB_PATH}/libCybertronLogApi.a)

	set(CURL_LIB_PATH ${CYBERTRON_PATH_ROOT}/3rdparty/libcurl)
	include_directories("${CURL_LIB_PATH}/include")
	target_link_libraries(${CMAKE_BUILD_NAME} debug ${CURL_LIB_PATH}/lib_linux_x64/libcurl.a)
	target_link_libraries(${CMAKE_BUILD_NAME} optimized ${CURL_LIB_PATH}/lib_linux_x64/libcurl.a)
	target_link_libraries(${CMAKE_BUILD_NAME} ${SIMONE_PATH_ROOT}/3rdparty/lz4/bin/x64_Release/liblz4.so)
ENDIF()

IF (WIN32)
	IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		add_custom_command(TARGET ${CMAKE_BUILD_NAME} 
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/debug/${CMAKE_BUILD_NAME}.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/${CMAKE_BUILD_NAME}.lib
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/debug/${CMAKE_BUILD_NAME}.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/${CMAKE_BUILD_NAME}.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/debug/${CMAKE_BUILD_NAME}.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/${CMAKE_BUILD_NAME}.lib
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/debug/${CMAKE_BUILD_NAME}.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/${CMAKE_BUILD_NAME}.dll
		)
		
	ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
		add_custom_command(TARGET ${CMAKE_BUILD_NAME} 
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/release/${CMAKE_BUILD_NAME}.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/${CMAKE_BUILD_NAME}.lib
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/release/${CMAKE_BUILD_NAME}.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/${CMAKE_BUILD_NAME}.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/release/${CMAKE_BUILD_NAME}.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/${CMAKE_BUILD_NAME}.lib
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/bin/release/${CMAKE_BUILD_NAME}.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/${CMAKE_BUILD_NAME}.dll
		)
	ENDIF()

ELSEIF (UNIX)
	 IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")

	ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
		IF (HDMAP)
			add_custom_command(TARGET ${CMAKE_BUILD_NAME} 
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/linux/release/libSSD.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libSSD.so
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/release/libHDMapModule.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libHDMapModule.so
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/release/libProtobufModule.a ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libProtobufModule.a
			)
		ENDIF()
		add_custom_command(TARGET ${CMAKE_BUILD_NAME} 
			POST_BUILD
		 	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/lib${CMAKE_BUILD_NAME}.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/lib${CMAKE_BUILD_NAME}.so
		 	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/lib/lib${CMAKE_BUILD_NAME}.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/lib${CMAKE_BUILD_NAME}.so
		 )
	ENDIF()
ENDIF()
