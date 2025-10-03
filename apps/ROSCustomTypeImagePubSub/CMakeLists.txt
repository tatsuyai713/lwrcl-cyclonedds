cmake_minimum_required(VERSION 3.18)

project(ROSCustomTypeImagePubSub LANGUAGES CXX VERSION 1)

message(STATUS "Configuring ROSCustomTypeImagePubSub...")

# --- C++ / warnings ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wnon-virtual-dtor
                    -Wno-deprecated-declarations -Wno-unused-result)

# --- Dependencies ---
find_package(yaml-cpp QUIET)

# Paths
set(UPDATE_HEADERS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_headers.sh")
set(IDLC_EXECUTABLE /opt/cyclonedds/bin/idlc)

# --- IDL (msg only) ---
set(IDL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/msg)
file(GLOB IDL_FILES "${IDL_ROOT}/*.idl")
if(IDL_FILES STREQUAL "")
  message(FATAL_ERROR "No IDL files found in ${IDL_ROOT}. Place *.idl directly under msg/.")
endif()

# --- Generated tree: <build>/generated/<ALIAS>/msg ---
# NOTE: set alias to match your #include path: ROSCustomTypeImagePubSub
set(GEN_PKG_DIR "${CMAKE_BINARY_DIR}/generated/ROSCustomTypeImagePubSub")
set(GEN_MSG_DIR "${GEN_PKG_DIR}/msg")

# Ensure WORKING_DIRECTORY exists first
add_custom_target(gen_msg_dir
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GEN_MSG_DIR}"
  COMMENT "Create ${GEN_MSG_DIR}"
  VERBATIM
)

# --- IDL -> C++ (per IDL) ---
set(GENERATED_SOURCES)
set(GENERATED_HEADERS)

foreach(IDL_FILE IN LISTS IDL_FILES)
  get_filename_component(IDL_BASENAME "${IDL_FILE}" NAME_WE)

  set(GEN_CPP "${GEN_MSG_DIR}/${IDL_BASENAME}.cpp")
  set(GEN_HPP "${GEN_MSG_DIR}/${IDL_BASENAME}.hpp")

  add_custom_command(
    OUTPUT "${GEN_CPP}"                                 # .hpp is a byproduct (mv by script)
    WORKING_DIRECTORY "${GEN_MSG_DIR}"
    COMMAND "${IDLC_EXECUTABLE}"
            -l cxx
            -fcase-sensitive
            -I "${IDL_ROOT}"
            -I "${CMAKE_CURRENT_SOURCE_DIR}"
            -I "/opt/cyclonedds-libs/include"
            "${IDL_FILE}"
    # minimal patch for return string (uses current PROJECT_NAME, which is fine)
    COMMAND ${CMAKE_COMMAND} -E env bash -c
            "sed -i 's|return \\\"${PROJECT_NAME}::msg::${IDL_BASENAME}\\\";|return \\\"${PROJECT_NAME}::msg::dds_::${IDL_BASENAME}_\\\";|g' \"${GEN_HPP}\""
    DEPENDS gen_msg_dir "${IDL_FILE}"
    BYPRODUCTS "${GEN_HPP}"
    COMMENT "idlc: Generating ${IDL_BASENAME} into ${GEN_MSG_DIR}"
    VERBATIM
  )

  list(APPEND GENERATED_SOURCES "${GEN_CPP}")
  list(APPEND GENERATED_HEADERS "${GEN_HPP}")
endforeach()

# Group target
add_custom_target(idl_gen ALL DEPENDS ${GENERATED_SOURCES})
add_dependencies(idl_gen gen_msg_dir)

# --- Step 2: run update_headers.sh after generation ---
add_custom_target(update_headers_target
    COMMAND ${CMAKE_COMMAND} -E echo "Running update_headers.sh in ${GEN_PKG_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env bash "${UPDATE_HEADERS_SCRIPT}"
    WORKING_DIRECTORY "${GEN_PKG_DIR}"
    DEPENDS idl_gen
    COMMENT "Updating headers & cpp includes (_cyclone rename etc.)"
    VERBATIM
)

# --- Step 3: create _final.cpp (copy after header updates) ---
set(FINAL_SRCS)
foreach(SRC IN LISTS GENERATED_SOURCES)
  get_filename_component(SRC_DIR  "${SRC}" DIRECTORY)
  get_filename_component(SRC_NAME "${SRC}" NAME_WE)
  set(FINAL_SRC "${SRC_DIR}/${SRC_NAME}_final.cpp")

  add_custom_command(
    OUTPUT "${FINAL_SRC}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC}" "${FINAL_SRC}"
    DEPENDS update_headers_target "${SRC}"
    COMMENT "Creating final copy: ${SRC_NAME}_final.cpp"
    VERBATIM
  )

  list(APPEND FINAL_SRCS "${FINAL_SRC}")
endforeach()

add_custom_target(final_sources DEPENDS ${FINAL_SRCS})

# --- Sources ---
file(GLOB ROSTYPEIMAGEPUBSUB_SOURCES_CXX "src/*.cxx")
file(GLOB ROSTYPEIMAGEPUBSUB_SOURCES_CPP "src/*.cpp")

# --- Config file (same behavior as reference) ---
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml
  COPYONLY
)

# --- Include directories (match reference + generated tree) ---
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
if(DEFINED ROS_DATA_TYPES_INCLUDE_PATH)
  include_directories(${ROS_DATA_TYPES_INCLUDE_PATH})
endif()

# Allow <ROSCustomTypeImagePubSub/msg/tf_message.hpp> during build
include_directories(
  ${CMAKE_BINARY_DIR}/generated
  ${GEN_PKG_DIR}
)

# --- Executable (name + install like reference) ---
add_executable(ROSCustomTypeImagePubSub
  ${ROSTYPEIMAGEPUBSUB_SOURCES_CXX}
  ${ROSTYPEIMAGEPUBSUB_SOURCES_CPP}
  ${FINAL_SRCS}
)

add_dependencies(ROSCustomTypeImagePubSub final_sources)
add_dependencies(ROSCustomTypeImagePubSub idl_gen)

# --- Link ---
target_include_directories(ROSCustomTypeImagePubSub
  SYSTEM PRIVATE
    /opt/qnx/cyclonedds/include
    /opt/qnx/cyclonedds-libs/include
)
target_link_libraries(ROSCustomTypeImagePubSub
  PRIVATE
    ddsc ddscxx
    lwrcl geometry_msgs sensor_msgs std_msgs builtin_interfaces yaml-cpp
)
if(TARGET yaml-cpp)
  target_link_libraries(ROSCustomTypeImagePubSub PRIVATE yaml-cpp)
endif()
if(OpenCV_FOUND)
  target_link_libraries(ROSCustomTypeImagePubSub PRIVATE ${OpenCV_LIBRARIES})
endif()

# --- Install (same as reference) ---
if(NOT DEFINED BIN_INSTALL_DIR)
  set(BIN_INSTALL_DIR .)
endif()

install(TARGETS ROSCustomTypeImagePubSub
    RUNTIME DESTINATION bin/ROSCustomTypeImagePubSub/${BIN_INSTALL_DIR}
)

install(FILES config/config.yaml
        DESTINATION bin/ROSCustomTypeImagePubSub/${BIN_INSTALL_DIR}/config
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
)

# --- Info ---
message(STATUS "IDL files (msg/):")
foreach(F IN LISTS IDL_FILES)
  get_filename_component(N "${F}" NAME)
  message(STATUS "  - ${N}")
endforeach()
message(STATUS "Generated tree: ${GEN_MSG_DIR}")
