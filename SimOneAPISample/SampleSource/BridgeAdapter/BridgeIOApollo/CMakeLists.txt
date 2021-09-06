add_executable(BridgeIOApollo "")

include_directories("${INCLUDE_DIR}")

IF (WIN32)
    add_definitions(-D_WIN32_WINNT=0x0501)
ENDIF()

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

OPTION(WITHOUT_HDMAP "Build SimOneIOAPI without HDMap" ON)
IF (WITHOUT_HDMAP)
	add_definitions(-DWITHOUT_HDMAP)
ENDIF()

IF (MSVC)
    string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
target_sources(BridgeIOApollo PRIVATE ${srcs})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

SET(APOLLO_ROOT /apollo)
set(CYBER_BRIDGE_ROOT ${APOLLO_ROOT}/bazel-bin/CyberBridge)

SetDefaultTargetProperties(BridgeIOApollo)

include_directories("${APOLLO_ROOT}/CyberBridge")

IF (WIN32)
    target_link_libraries(BridgeIOApollo  ${LIB_DIR}/SimOneIOAPI.lib ${LIB_DIR}/SSD.lib ${LIB_DIR}/HDMapModule.lib  ${LIB_DIR}/ProtobufModule.lib)
    add_custom_command(TARGET BridgeIOApollo
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SimOneIOAPI.dll ${BINARY_OUT_PATH}/SimOneIOAPI.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/SSD.dll ${BINARY_OUT_PATH}/SSD.dll
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/HDMapModule.dll ${BINARY_OUT_PATH}/HDMapModule.dll
        )
ELSEIF (UNIX)
        target_link_libraries(BridgeIOApollo debug ${LIB_DIR}/libSimOneIOAPI.so -lpthread)
        target_link_libraries(BridgeIOApollo debug /Cybertron2/Modules/Foundation/Build/build_debug/lib/libprotobuf.a)
        
        #target_link_libraries(BridgeIOApollo optimized /Cybertron2/Modules/Foundation/Build/build_debug/lib/libprotobuf.a -lpthread -lprotobuf)
        target_link_libraries(BridgeIOApollo debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        target_link_libraries(BridgeIOApollo optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
        # target_link_libraries(BridgeIOApollo debug ${LIB_DIR}/libSSD.so)
        # target_link_libraries(BridgeIOApollo optimized ${LIB_DIR}/libSSD.so -lpthread)
        # target_link_libraries(BridgeIOApollo debug ${LIB_DIR}/libHDMapModule.so)
        # target_link_libraries(BridgeIOApollo optimized ${LIB_DIR}/libHDMapModule.so)
        target_link_libraries(BridgeIOApollo debug ${LIB_DIR}/libApolloCyber.so)
        target_link_libraries(BridgeIOApollo optimized ${CYBER_BRIDGE_ROOT}/libApolloCyber.so)
        #target_link_libraries(BridgeIOApollo optimized ${LIB_DIR}/libSimOneIOAPI.so -lpthread -lprotobuf)

        add_custom_command(TARGET BridgeIOApollo
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneIOAPI.so ${BINARY_OUT_PATH}/libSimOneIOAPI.so
        # COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
        # COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so
        )
ENDIF()
