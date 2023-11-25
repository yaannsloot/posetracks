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

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/me)

message("Looking for modules...")

FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(me/${subdir})
ENDFOREACH()

message("done.")

set(SOURCES_MOTIONENGINE_PYBIND
	${CMAKE_CURRENT_SOURCE_DIR}/python.cpp
)

set(SOURCES_ME_POSE_TEST
	${CMAKE_CURRENT_SOURCE_DIR}/testing.cpp
)

set(SOURCES_MOTIONENGINE ${SOURCES_MOTIONENGINE} PARENT_SCOPE)
set(SOURCES_MOTIONENGINE_PYBIND ${SOURCES_MOTIONENGINE_PYBIND} PARENT_SCOPE)
set(SOURCES_ME_POSE_TEST ${SOURCES_ME_POSE_TEST} PARENT_SCOPE)
set(MODULE_LIST ${MODULE_LIST} PARENT_SCOPE)