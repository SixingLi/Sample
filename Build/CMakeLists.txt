cmake_minimum_required(VERSION 3.8.0)

project(SimOneAPI VERSION 3.0.0 LANGUAGES C CXX)

set(OutPutDir "SimOneAPISample")
set(OutPutDirSDK "SimOneAPIInternalTest")

IF (UNIX)
    set(CMAKE_C_COMPILER "/usr/bin/clang")
    set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -I/usr/include/libcxxabi")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -stdlib=libc++ -lc++abi -rpath=./")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)


OPTION(WITHOUT_HDMAP "Build ${CMAKE_BUILD_NAME} without HDMap" OFF)
IF (WITHOUT_HDMAP)
	add_definitions(-DWITHOUT_HDMAP)
ENDIF()

OPTION(WITHOUT_SENSOR "Build ${CMAKE_BUILD_NAME} without Sensor" OFF)
IF (WITHOUT_SENSOR)
	add_definitions(-DWITHOUT_SENSOR)
ENDIF()

OPTION(WITHOUT_PNC "Build ${CMAKE_BUILD_NAME} without Planning and Control" OFF)
IF (WITHOUT_PNC)
	add_definitions(-DWITHOUT_PNC)
ENDIF()

OPTION(WITHOUT_V2X "Build ${CMAKE_BUILD_NAME} without Planning and Control" OFF)
IF (WITHOUT_V2X)
	add_definitions(-DWITHOUT_V2X)
ENDIF()

set(Boost_USE_STATIC_LIBS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

set(CYBERTRON_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(SIMONE_PATH_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(SIMONE_PATH_FOUNDATION ${CMAKE_CURRENT_LIST_DIR}/../../Foundation)
set(SIMONE_API_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)

function(SetDefaultTargetProperties target)
    set_target_properties(${target} PROPERTIES
        CXX_STANDARD 14
    )
    set_target_properties(${target} PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
endfunction()

add_subdirectory(${SIMONE_API_ROOT}/Source Source)

include_directories("${SIMONE_PATH_ROOT}/3rdparty")

IF (UNIX)
set(PythonLibs ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/bin)
ELSE (UNIX)
set(PythonLibs ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36)
ENDIF()
find_package(PythonLibs 3.6 REQUIRED)

add_custom_target(autoconfig ALL DEPENDS)

IF (WIN32)
	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Debug)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Release)
	
    add_custom_command(TARGET autoconfig
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDir}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDir}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneServiceAPI.h
	)

	IF (NOT WITHOUT_SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneSensorAPI.h 
		)
		IF (NOT WITHOUT_V2X)
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDir}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.lib

				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDir}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.lib
			)
		ENDIF()	
	ENDIF()	

		IF (NOT WITHOUT_SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneSensorAPI.h 
		)
		IF (NOT WITHOUT_V2X)
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneV2XAPI.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDir}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.lib

				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDir}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.lib
			)
		ENDIF()	
	ENDIF()	
	
	IF (NOT WITHOUT_HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDir}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDir}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneHDMapAPI.h 
		)
	ENDIF()	

	IF (NOT WITHOUT_PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOnePNCAPI.h
		)
	ENDIF()	
###-----------------------------------------------------------------------------------------------------------------------------###
    add_custom_command(TARGET autoconfig
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneServiceAPI.h
	)

	IF (NOT WITHOUT_SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneSensorAPI.h 
		)
		IF (NOT WITHOUT_V2X)
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneV2XAPI.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/tsf.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/tsf.lib

				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/V2X_ASN_New4Span.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/V2X_ASN_New4Span.lib
			)
		ENDIF()	
	ENDIF()	
	
	IF (NOT WITHOUT_HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneHDMapAPI.h 
		)
	ENDIF()	

	IF (NOT WITHOUT_PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOnePNCAPI.h
		)
	ENDIF()	
                        
ELSEIF (UNIX)
    IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
			SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_debug/lib)
			add_custom_command(TARGET autoconfig
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${FOUNDATION_DBG_LIB_PATH}/libgtest.a ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libgtest.a
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_debug/lib/libSource.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libSource.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_debug/lib/libSimOneStreamingIOAPI.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libSimOneStreamingIOAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavcodec.so.58 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavcodec.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavformat.so.58 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavformat.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavutil.so.56 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavutil.so.56
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswresample.so.3 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libswresample.so.3
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswscale.so.5 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libswscale.so.5
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libx264.so.148 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libx264.so.148
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen/ ${SIMONE_API_ROOT}/Dependency/Workshop/AVP/eigen/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/gtest/include/gtest/ ${SIMONE_API_ROOT}/Dependency/include/gtest/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/linux/release/ ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/linux/release/ ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/Dependency/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/Dependency/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/Dependency/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneIOStruct.h ${SIMONE_API_ROOT}/Dependency/include/Service/SimOneIOStruct.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneNetAPI.h ${SIMONE_API_ROOT}/Dependency/include/SimOneNetAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/Dependency/include/SimOneStreamingAPI.h
				 
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_debug/pySimOneIO/pySimOneIO.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/pySimOneIO.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_debug/pySimOneV2X/pySimOneV2X.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/pySimOneV2X.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneIOStruct.py ${SIMONE_API_ROOT}/Dependency/lib/Linux64/SimOneIOStruct.py
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/osi ${SIMONE_API_ROOT}/Dependency/OSI/src/osi/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/Dependency/include/nlohmann/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/google ${SIMONE_API_ROOT}/Dependency/include/google/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../Bridges/BridgeV2X/CybertronBridgeV2X/header ${SIMONE_API_ROOT}/Dependency/include/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/ ${SIMONE_API_ROOT}/SimOneSMAPISample/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/include/ ${SIMONE_API_ROOT}/SimOneSMAPISample/include/
			)
                        
    ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
			SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build_release/lib)
			add_custom_command(TARGET autoconfig
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${FOUNDATION_REL_LIB_PATH}/libgtest.a ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libgtest.a
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_release/lib/libSource.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libSource.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_release/lib/libSimOneStreamingIOAPI.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libSimOneStreamingIOAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/linux/release/ ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/linux/release/ ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/Dependency/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/Dependency/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/Dependency/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneIOStruct.h ${SIMONE_API_ROOT}/Dependency/include/Service/SimOneIOStruct.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneNetAPI.h ${SIMONE_API_ROOT}/Dependency/include/SimOneNetAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/Dependency/include/SimOneStreamingAPI.h			
            COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_release/pySimOneIO/pySimOneIO.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/pySimOneIO.so
            COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Build/build_release/pySimOneV2X/pySimOneV2X.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/pySimOneV2X.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/SimOneIOStruct.py ${SIMONE_API_ROOT}/Dependency/lib/Linux64/SimOneIOStruct.py
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/OSIExternal/lib/ubuntu/libprotobuf.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/OSIExternal/lib/ubuntu/libopen_simulation_interface.so ${SIMONE_API_ROOT}/Dependency/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/osi ${SIMONE_API_ROOT}/Dependency/OSI/src/osi/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/Dependency/include/nlohmann/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/google ${SIMONE_API_ROOT}/Dependency/include/google/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../Bridges/BridgeV2X/CybertronBridgeV2X/header ${SIMONE_API_ROOT}/Dependency/include/

			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavcodec.so.58 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavcodec.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavformat.so.58 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavformat.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavutil.so.56 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libavutil.so.56
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswresample.so.3 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libswresample.so.3
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswscale.so.5 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libswscale.so.5
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libx264.so.148 ${SIMONE_API_ROOT}/Dependency/lib/Linux64/libx264.so.148
		  
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen/ ${SIMONE_API_ROOT}/Dependency/Workshop/AVP/eigen/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/gtest/include/gtest/ ${SIMONE_API_ROOT}/Dependency/include/gtest/
            )
    ENDIF()
ENDIF()
