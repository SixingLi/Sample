cmake_minimum_required(VERSION 3.8.0)

IF (MSVC)
    string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REGEX REPLACE "/RTC[^ ]*" "" CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

IF (UNIX)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -fPIC -stdlib=libc++ -Wno-inconsistent-missing-override -I/usr/include/libcxxabi")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")
ENDIF()

if (CMAKE_CL_64)
    set(PLATFORM_DIR win64)
else (CMAKE_CL_64)
    set(PLATFORM_DIR win32)
endif(CMAKE_CL_64)

IF (UNIX)
	set(PLATFORM_DIR linux)
ENDIF()

IF (UNIX)
set(PYTHON_LIBRARY ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/lib/libpython3.6m.so)
set(PYTHON_INCLUDE_PATH ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/include/python3.6m)
set(PYTHON_INCLUDE_DIRS ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/include/python3.6m)
ELSE (UNIX)
set(PYTHON_LIBRARY ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/libs/python36.lib)
set(PYTHON_INCLUDE_PATH ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/include)
set(PYTHON_INCLUDE_DIRS ${SIMONE_PATH_ROOT}/3rdparty/artifactory/python36/include)
ENDIF()


set(SSD_INCLUDE ${SIMONE_PATH_ROOT}/3rdparty/SSD/include)
set(Boost_INCLUDE_DIRS ${SIMONE_PATH_ROOT}/3rdparty/boost_python/include)
set(HDMap_INCLUDE_DIRS ${SIMONE_PATH_ROOT}/3rdparty/hdmap/include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${HDMap_INCLUDE_DIRS})
include_directories(${SSD_INCLUDE})

IF (UNIX)
set(BOOSTPYTHON_LIB_NAME libboost_python36.a)
set(HDMAP_LIB_NAME libHDMapModule.so)
ELSE (UNIX)
set(BOOSTPYTHON_LIB_NAME libboost_python-vc141-mt-x64-1_67.lib)
set(HDMAP_LIB_NAME HDMapModule.lib)
ENDIF()

######################################################################
########################### HDMapAPI ##############################
######################################################################
python_add_module(HDMapAPI SHARED ${CMAKE_CURRENT_LIST_DIR}/bindings.cpp)
target_include_directories(HDMapAPI PRIVATE 
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${HDMap_INCLUDE_DIRS}
	${SSD_INCLUDE}
	)
include_directories(${SIMONE_API_ROOT}/SourceCpp/APIService)
include_directories(${SIMONE_API_ROOT}/SourceCpp/HDMap)
include_directories(${SIMONE_API_ROOT}/SourceCpp/Sensor/Task)

target_compile_definitions(HDMapAPI PRIVATE BOOST_PYTHON_STATIC_LIB)

target_link_libraries(HDMapAPI ${CMAKE_BUILD_NAME}
    debug ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/${PLATFORM_DIR}/release/${HDMAP_LIB_NAME}
	optimized ${SIMONE_PATH_ROOT}/3rdparty/hdmap/lib/${PLATFORM_DIR}/release/${HDMAP_LIB_NAME}
	${PYTHON_LIBRARY}
	${SIMONE_PATH_ROOT}/3rdparty/boost_python/lib/${PLATFORM_DIR}/release/${BOOSTPYTHON_LIB_NAME}
	)
	
IF (WIN32)
	IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		add_custom_command(TARGET HDMapAPI
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/Debug/HDMapAPI.pyd ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/HDMapAPI.pyd
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/Debug/HDMapAPI.pyd ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/HDMapAPI.pyd
		)
	ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
		add_custom_command(TARGET HDMapAPI
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/Release/HDMapAPI.pyd ${SIMONE_API_ROOT}/${OutPutDir}/lib/Win64/HDMapAPI.pyd
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/Release/HDMapAPI.pyd ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Win64/HDMapAPI.pyd
		)
	ENDIF()
ELSEIF(UNIX)
	IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
		add_custom_command(TARGET HDMapAPI
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/HDMapAPI.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/HDMapAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/HDMapAPI.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/HDMapAPI.so
		)
	ELSEIF (${CMAKE_BUILD_TYPE} STREQUAL "Release")
		add_custom_command(TARGET HDMapAPI
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/HDMapAPI.so ${SIMONE_API_ROOT}/${OutPutDir}/lib/Linux64/HDMapAPI.so
			COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_BINARY_DIR}/HDMapAPI/HDMapAPI.so ${SIMONE_API_ROOT}/${OutPutDirSDK}/lib/Linux64/HDMapAPI.so
		)
	ENDIF()
ENDIF()
