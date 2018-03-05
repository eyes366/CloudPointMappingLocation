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

include_directories(./src)

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
									                ./src/CommonHeader.h)
target_link_libraries (LocationOprAPI ${PCL_LIBRARIES} ${PCAP_LIBRARIES})

add_executable(main_localization ./src/main_localization.cpp
                ./src/ImuCalc.cpp
                ./src/ImuCalc.h
                ./src/DataSimulator.h
                ./src/DataSimulator.cpp
                ./src/LocationOprAPI.h)
target_link_libraries (main_localization ${PCL_LIBRARIES} ${PCAP_LIBRARIES} LocationOprAPI)

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
