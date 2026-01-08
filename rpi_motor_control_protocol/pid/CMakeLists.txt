# pid/CMakeLists.txt

#######
# pid #
#######

add_library(pid STATIC pid.c)
target_include_directories(pid PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)
