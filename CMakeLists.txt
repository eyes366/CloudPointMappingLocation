project(PclTest C CXX)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#pcl
add_definitions(-DNOMINMAX)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#pcap
set(PCAP_DIR ${PclTest_SOURCE_DIR}/third/WpdPack)
set(PCAP_DEFINES -DHAVE_PCAP -DWIN32)
set(PCAP_INCLUDE_DIRS ${PCAP_DIR}/include)
set(PCAP_LIBRARIES ${PCAP_DIR}/Lib/x64/wpcap.lib ${PCAP_DIR}/Lib/x64/Packet.lib)
include_directories(${PCAP_INCLUDE_DIRS})
add_definitions(${PCAP_DEFINES})

#Qt4
#FIND_PACKAGE(Qt4 COMPONENTS QtCore QtXml QtOpenGL QtGui QtNetwork REQUIRED)
#INCLUDE(${QT_USE_FILE})
# QT4_WRAP_CPP(MOCS ${moc_list})
# QT4_WRAP_UI(UIS ${ui_list})
# QT4_ADD_RESOURCES(QRCS ${PROJECT_SOURCE_DIR}/resource/resource.qrc)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${QT_LIBRARIES})


#cc
#set(CC_DIR ${PROJECT_SOURCE_DIR}/third/CC-2.6.3.1)
#find_path(CC_DIR "")
#IF(CC_DIR)
#	MESSAGE(STATUS "Found CC_DIR: ${CC_DIR}")
#	include_directories(${CC_DIR}/CCCORE/include)
#	include_directories(${CC_DIR}/CCFbo/include)
#	include_directories(${CC_DIR}/Glew/include)
#	include_directories(${CC_DIR}/qCC_db/include)
#	target_link_libraries(${PROJECT_NAME} debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib)
#	target_link_libraries(${PROJECT_NAME} debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib)
#	target_link_libraries(${PROJECT_NAME} debug ${CC_DIR}/CCFbo/lib/CC_FBO_LIBd.lib optimized ${CC_DIR}/CCFbo/lib/CC_FBO_LIB.lib)
#	target_link_libraries(${PROJECT_NAME} debug ${CC_DIR}/Glew/lib/GLEW_LIBd.lib optimized ${CC_DIR}/Glew/lib/GLEW_LIB.lib)
#ELSE(CC_DIR)
#	MESSAGE(FATAL_ERROR "Could not find CC")
#ENDIF(CC_DIR)

include_directories(./src)

#main_evaluation
add_executable(main_evaluation ./src/main_evaluation.cpp
								./src/ImuNav.h
								./src/ImuNav.cpp)

#main_matching
add_executable(main_matching ./src/main_matching.cpp)
target_link_libraries(main_matching 
									${PCL_LIBRARIES}
#									${QT_LIBRARIES} 
#									debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib 
#									optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib 
#									debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib 
#									optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib
									)

#LocationOprAPI
#SET(LOCATION_OPR_API optimized LocationOprAPI debug LocationOprAPId)
add_library(LocationOprAPI SHARED ./src/LocationOprAPI.cpp
													./src/LocationOprAPI.h
													./src/hdl_grabber_.cpp
									                ./src/hdl_grabber_.h
									                ./src/vlp_grabber_.cpp
									                ./src/vlp_grabber_.h
									                ./src/ImuCalc.cpp
									                ./src/ImuCalc.h
									                ./src/VelodyneOpr.h
									                ./src/VelodyneOpr.cpp
									                ./src/MapConstruct.h
									                ./src/MapConstruct.cpp
									                ./src/LocationOpr.cpp
									                ./src/LocationOpr.h
									                ./src/common.h
									                ./src/common.cpp
									                ./src/CommonHeader.h
#													./src/cc_algorithm.h
#													./src/cc_algorithm.cpp
													)
target_link_libraries (LocationOprAPI 
							${PCL_LIBRARIES} 
							${PCAP_LIBRARIES}
#							${QT_LIBRARIES} 
#							debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib 
#							optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib 
#							debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib 
#							optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib
							)

#main_localization
add_executable(main_localization ./src/main_localization.cpp
                ./src/ImuCalc.cpp
                ./src/ImuCalc.h
                ./src/DataSimulator.h
                ./src/DataSimulator.cpp
                ./src/LocationOprAPI.h)
target_link_libraries (main_localization 
							${PCL_LIBRARIES} 
							${PCAP_LIBRARIES} 
							LocationOprAPI
#							${QT_LIBRARIES} 
#							debug ${CC_DIR}/qCC_db/lib/QCC_DB_LIBd.lib 
#							optimized ${CC_DIR}/qCC_db/lib/QCC_DB_LIB.lib 
#							debug ${CC_DIR}/CCCORE/lib/CC_CORE_LIBd.lib 
#							optimized ${CC_DIR}/CCCORE/lib/CC_CORE_LIB.lib
							)

#main_mapping
add_executable(main_mapping ./src/main.cpp
                ./src/hdl_grabber_.cpp
                ./src/hdl_grabber_.h
                ./src/vlp_grabber_.cpp
                ./src/vlp_grabber_.h
                ./src/VelodyneOpr.h
                ./src/VelodyneOpr.cpp
                ./src/MapConstruct.h
                ./src/MapConstruct.cpp
                ./src/MappingOpr.cpp
                ./src/MappingOpr.h
                ./src/ImuNav.h
                ./src/ImuNav.cpp
                ./src/ImuCalc.h
                ./src/ImuCalc.cpp
                ./src/common.h
                ./src/common.cpp)
target_link_libraries (main_mapping ${PCL_LIBRARIES} ${PCAP_LIBRARIES})

set(CMAKE_INSTALL_PREFIX ${PclTest_SOURCE_DIR}/install)
INSTALL(FILES ./src/LocationOprAPI.h DESTINATION "include")
INSTALL(TARGETS LocationOprAPI LIBRARY DESTINATION "lib"
                      			 ARCHIVE DESTINATION "lib"
                      			 RUNTIME DESTINATION "bin"
                      			 COMPONENT library)