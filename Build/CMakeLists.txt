cmake_minimum_required(VERSION 3.8.0)

project(SimOneAPI VERSION 3.0.0 LANGUAGES C CXX)

set(OutPutDir "InternalTest")
set(OutPutDirSDK "Samples")

IF (UNIX)
	set(CMAKE_C_COMPILER "/usr/bin/clang")
	set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -I/usr/include/libcxxabi")
	set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -stdlib=libc++ -lc++abi -rpath=./")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

add_definitions(-D_ITERATOR_DEBUG_LEVEL=0)

OPTION(HDMAP "Build ${CMAKE_BUILD_NAME} HDMap" ON)
IF (HDMAP)
	add_definitions(-DHDMAP)
ENDIF()

OPTION(SENSOR "Build ${CMAKE_BUILD_NAME} Sensor" ON)
IF (SENSOR)
	add_definitions(-DSENSOR)
ENDIF()

OPTION(PNC "Build ${CMAKE_BUILD_NAME} Planning and Control" ON)
IF (PNC)
	add_definitions(-DPNC)
ENDIF()

OPTION(V2X "Build ${CMAKE_BUILD_NAME} V2X" ON)
IF (V2X)
	add_definitions(-DV2X)
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

include_directories("${SIMONE_PATH_ROOT}/3rdparty")
IF (UNIX)
set(PythonLibs ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/bin)
ELSE ()
set(PythonLibs ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36)
ENDIF()
find_package(PythonLibs 3.6 REQUIRED)

add_custom_target(autoconfig ALL DEPENDS)

add_subdirectory(${SIMONE_API_ROOT}/SourceCpp SourceCpp)
add_subdirectory(${SIMONE_API_ROOT}/InternalTest/TestAPI TestAPI)
add_dependencies(TestAPI ${CMAKE_BUILD_NAME})

IF (HDMAP)
	add_subdirectory(${SIMONE_API_ROOT}/SourcePython/HDMapAPI HDMapAPI)
	add_dependencies(HDMapAPI ${CMAKE_BUILD_NAME})
ENDIF()

IF (WIN32)

	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Debug)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Release)
	
    	add_custom_command(TARGET autoconfig
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDir}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen ${SIMONE_API_ROOT}/${OutPutDir}/include/eigen/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDir}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneServiceAPI.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneServiceAPI.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneServiceAPI.py
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneIOStruct.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneIOStruct.py
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/PythonAPISample.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/PythonAPISample.py
	)

	IF (SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneSensorAPI.h 
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneSensorAPI.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneSensorAPI.py
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/include/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneStreamingAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/SimOneStreamingAPI.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneStreamingAPI.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/SimOneStreamingAPI.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneStreamingAPI.lib
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avcodec-58.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/avcodec-58.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avformat-58.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/avformat-58.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avutil-56.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/avutil-56.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/swresample-3.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/swresample-3.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/swscale-5.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/swscale-5.dll
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneStreamingAPI.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneStreamingAPI.py
		)
		IF (V2X)
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneV2XAPI.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneV2XAPI.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOneV2XAPI.py
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDir}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/tsf.lib
	
				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDir}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.dll ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.lib ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/V2X_ASN_New4Span.lib
			)
		ENDIF()	
	ENDIF()	
	
	IF (HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDir}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDir}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/MHDMap.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneHDMapAPI.h 
		)
	ENDIF()	

	IF (PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOnePNCAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOnePNCAPI.py ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/SimOnePNCAPI.py
		)
	ENDIF()	
###-----------------------------------------------------------------------------------------------------------------------------###
    add_custom_command(TARGET autoconfig
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/eigen/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneServiceAPI.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneServiceAPI.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneServiceAPI.py
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneIOStruct.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneIOStruct.py
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/PythonAPISample.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/PythonAPISample.py
	)

	IF (SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneSensorAPI.h 
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/include/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneStreamingAPI.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/SimOneStreamingAPI.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneStreamingAPI.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/SimOneStreamingAPI.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneStreamingAPI.lib
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avcodec-58.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/avcodec-58.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avformat-58.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/avformat-58.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/avutil-56.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/avutil-56.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/swresample-3.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/swresample-3.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/bin/swscale-5.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/swscale-5.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneSensorAPI.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneSensorAPI.py
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneStreamingAPI.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneStreamingAPI.py
		)
		IF (V2X)
		message("---------------------------------------------------------------------------")
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneV2XAPI.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOneV2XAPI.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOneV2XAPI.py
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/tsf.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/win64/release/tsf.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/tsf.lib

				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.dll ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/V2X_ASN_New4Span.dll
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/win64/release/V2X_ASN_New4Span.lib ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/V2X_ASN_New4Span.lib
			)
		ENDIF()	
	ENDIF()	
	
	IF (HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/MHDMap.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneHDMapAPI.h 
		)
	ENDIF()	

	IF (PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOnePNCAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourcePython/CommonAPI/SimOnePNCAPI.py ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/SimOnePNCAPI.py
		)
	ENDIF()
ELSEIF (UNIX)
############ UNIX-----------------------------------------------------------------------------------------------------------------------------------test#############

	SET(FOUNDATION_DBG_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Debug)
	SET(FOUNDATION_REL_LIB_PATH ${SIMONE_PATH_FOUNDATION}/Build/build/lib/Release)

    	add_custom_command(TARGET autoconfig
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/eigen/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneServiceAPI.h

		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/nlohmann ${SIMONE_API_ROOT}/${OutPutDir}/include/nlohmann/
		COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/eigen ${SIMONE_API_ROOT}/${OutPutDir}/include/eigen/
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/Service/SimOneIOStruct.h ${SIMONE_API_ROOT}/${OutPutDir}/include/Service/SimOneIOStruct.h
		COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/APIService/SimOneServiceAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneServiceAPI.h
	)

	IF (SENSOR)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneSensorAPI.h 
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_API_ROOT}/../../3rdparty/streaming/include/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneStreamingAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/libSimOneStreamingAPI.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libSimOneStreamingAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavcodec.so.58 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libavcodec.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavformat.so.58 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libavformat.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavutil.so.56 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libavutil.so.56
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswresample.so.3 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libswresample.so.3
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswscale.so.5 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libswscale.so.5
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libx264.so.148 ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libx264.so.148

			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneSensorAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneSensorAPI.h 
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_API_ROOT}/../../3rdparty/streaming/include/SimOneStreamingAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneStreamingAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SIMONE_API_ROOT}/../../3rdparty/streaming/lib/libSimOneStreamingAPI.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libSimOneStreamingAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavcodec.so.58 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libavcodec.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavformat.so.58 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libavformat.so.58
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libavutil.so.56 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libavutil.so.56
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswresample.so.3 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libswresample.so.3
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libswscale.so.5 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libswscale.so.5
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/artifactory/ffmpeg/lib/libx264.so.148 ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libx264.so.148
		)

		IF (V2X)
			add_custom_command(TARGET autoconfig
			POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneV2XAPI.h 
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/linux/release/libtsf.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libtsf.so
				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/linux/release/libV2X_ASN_New4Span.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/libV2X_ASN_New4Span.so

				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneV2XAPI.h 
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDir}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/linux/release/libtsf.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libtsf.so
				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDir}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/linux/release/libV2X_ASN_New4Span.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libV2X_ASN_New4Span.so
			)
		ENDIF()
	ENDIF()	
	
	IF (HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			#${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/MHDMap.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOneHDMapAPI.h 

			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDir}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDir}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/MHDMap.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneHDMapAPI.h 
		)

	ENDIF()	

	IF (PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/SimOnePNCAPI.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/SourceCpp/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOnePNCAPI.h
		)
	ENDIF()
ENDIF()

