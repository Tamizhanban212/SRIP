execute_process(COMMAND "/home/tamizhanban/Documents/SRIP/build/kdl_parser/kdl_parser_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tamizhanban/Documents/SRIP/build/kdl_parser/kdl_parser_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
