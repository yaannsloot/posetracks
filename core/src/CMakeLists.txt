MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/modules)

message("Looking for modules...")

FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(modules/${subdir})
ENDFOREACH()

message("done.")

set(SOURCES_POSETRACKS_PYBIND
	${CMAKE_CURRENT_SOURCE_DIR}/python.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/bpy_data_impl.cpp
)

if(NOT UNIX AND NOT APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  set(SOURCES_POSETRACKS_PYBIND ${SOURCES_POSETRACKS_PYBIND} ${CMAKE_CURRENT_SOURCE_DIR}/winmain.cpp)
endif()

set(SOURCES_POSETRACKS ${SOURCES_POSETRACKS} PARENT_SCOPE)
set(SOURCES_POSETRACKS_PYBIND ${SOURCES_POSETRACKS_PYBIND} PARENT_SCOPE)
set(MODULE_LIST ${MODULE_LIST} PARENT_SCOPE)