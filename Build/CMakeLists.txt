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

OPTION(WITHOUT_V2X "Build ${CMAKE_BUILD_NAME} without V2X" OFF)
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
#add_subdirectory(${SIMONE_API_ROOT}/SimOneAPIInternalTest/APITest APITest)

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
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/MHDMap.h
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
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDirSDK}/include/public/MHDMap.h
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
############ UNIX-----------------------------------------------------------------------------------------------------------------------------------test#############
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
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/Sensor/SimOneV2XAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneV2XAPI.h 
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/include/tsf.h ${SIMONE_API_ROOT}/${OutPutDir}/include/tsf.h
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/tsf/lib/linux/release/libtsf.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libtsf.so

				COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/include ${SIMONE_API_ROOT}/${OutPutDir}/include/v2xinclude/
				COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/v2x_asn/v2x_I_New4Span/lib/linux/release/libV2X_ASN_New4Span.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/libV2X_ASN_New4Span.so
			)
		ENDIF()		
	ENDIF()	
	
	IF (NOT WITHOUT_HDMAP)
		add_custom_command(TARGET autoconfig
		POST_BUILD #${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/win64/${CMAKE_BUILD_TYPE}/ ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/win64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/lib/linux/release/ ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/SSD/include/SSD ${SIMONE_API_ROOT}/${OutPutDir}/include/SSD/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/libexport.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/libexport.h
			COMMAND ${CMAKE_COMMAND} -E copy_directory  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/common/ ${SIMONE_API_ROOT}/${OutPutDir}/include/public/common/
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/../../3rdparty/hdmap/include/public/MHDMap.h ${SIMONE_API_ROOT}/${OutPutDir}/include/public/MHDMap.h
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/HDMap/SimOneHDMapAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOneHDMapAPI.h 
		)
	ENDIF()	

	IF (NOT WITHOUT_PNC)
		add_custom_command(TARGET autoconfig
		POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${SIMONE_API_ROOT}/Source/PNC/SimOnePNCAPI.h ${SIMONE_API_ROOT}/${OutPutDir}/include/SimOnePNCAPI.h
		)
	ENDIF()
ENDIF()

