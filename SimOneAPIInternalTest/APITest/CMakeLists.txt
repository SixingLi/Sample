add_executable(OpenSimOneAPITest "")

include_directories("${SIMONE_API_ROOT}/Source/APIService")
include_directories("${SIMONE_API_ROOT}/Source/Sensor")
include_directories("${SIMONE_PATH_ROOT}/3rdparty")
include_directories("${SIMONE_PATH_ROOT}/3rdparty/SSD/include")
include_directories("${SIMONE_PATH_ROOT}/3rdparty/hdmap/include")
include_directories("${SIMONE_PATH_ROOT}/3rdparty/hdmap/include/modules")
include_directories("${SIMONE_PATH_ROOT}/3rdparty/gtest/include")
include_directories("${SIMONE_PATH_ROOT}/3rdparty/v2x_asn/v2x_I_New4Span/include")

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
target_sources(OpenSimOneAPITest PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SetDefaultTargetProperties(OpenSimOneAPITest)

IF (WIN32)
	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Debug)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Release)

	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/${BUILD_TYPE}.lib)
	target_link_libraries(OpenSimOneAPITest debug ${FOUNDATION_DBG_LIB_PATH}/protobuf.lib)
    target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/win64/debug/SSD.lib)
	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/debug/HDMapModule.lib)
	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/debug/ProtobufModule.lib)
	
	target_link_libraries(OpenSimOneAPITest optimized ${FOUNDATION_REL_LIB_PATH}/protobuf.lib)
    target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/win64/release/SSD.lib)
	target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/release/HDMapModule.lib)
	target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/win64/release/ProtobufModule.lib)
	
ELSEIF (UNIX)
	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_debug/lib)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_release/lib)
	target_link_libraries(OpenSimOneAPITest debug ${FOUNDATION_DBG_LIB_PATH}/libgtest.a -pthread -lrt)
	target_link_libraries(OpenSimOneAPITest debug ${FOUNDATION_DBG_LIB_PATH}/libprotobuf.a)
    target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/linux/debug/libSSD.so)
	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/debug/libHDMapModule.so)
	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/debug/libProtobufModule.a)

	target_link_libraries(OpenSimOneAPITest optimized ${FOUNDATION_REL_LIB_PATH}/libgtest.a -pthread  -lrt)
	target_link_libraries(OpenSimOneAPITest optimized ${FOUNDATION_REL_LIB_PATH}/libprotobuf.a)
    target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/SSD/lib/linux/release/libSSD.so)
	target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/release/libHDMapModule.so)
	target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/linux/release/libProtobufModule.a)
	target_link_libraries(OpenSimOneAPITest debug ${SIMONE_PATH_ROOT}/3rdparty/libcurl/lib_linux_x64/libcurl.a)
	target_link_libraries(OpenSimOneAPITest debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
	target_link_libraries(OpenSimOneAPITest optimized ${SIMONE_PATH_ROOT}/3rdparty/libcurl/lib_linux_x64/libcurl.a)
	target_link_libraries(OpenSimOneAPITest optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
ENDIF()

# IF (MSVC)
	# SET_PROPERTY(TARGET OpenSimOneAPITest PROPERTY FOLDER "Test")
# ENDIF()

# add_custom_command(TARGET OpenSimOneAPITest PRE_BUILD
                   # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/blob.dat $<TARGET_FILE_DIR:OpenSimOneAPITest>/blob.dat)
# add_custom_command(TARGET OpenSimOneAPITest PRE_BUILD
                   # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/blob.dat ${SIMONE_API_ROOT}/Build/build/OpenSimOneAPITest/blob.dat)
