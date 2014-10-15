file(GLOB_RECURSE UI_SOURCES ${PROJECT_SOURCE_DIR}/src/*.ui)
qt4_wrap_ui(UI_H ${UI_SOURCES})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB_RECURSE ALL_HEADERS ${PROJECT_SOURCE_DIR}/src/*.h)
set(MOC_H)
foreach(header ${ALL_HEADERS})
  file(STRINGS "${header}" lines REGEX "Q_OBJECT")
  if(lines)
    list(APPEND MOC_H "${header}")
  endif()
endforeach()
qt4_wrap_cpp(MOC_CPP ${MOC_H})

add_library(roah_devices_qt SHARED ${UI_H} ${MOC_CPP})

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
