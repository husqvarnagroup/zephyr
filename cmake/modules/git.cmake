# SPDX-License-Identifier: Apache-2.0

include_guard(GLOBAL)

find_package(Git QUIET)

# Usage:
#   git_describe(<dir> <output> MATCH_GLOB_PREFIX <match_glob> <dirty_flag>)
#
# Helper function to get a short GIT description associated with a directory.
# OUTPUT is set to the output of `git describe --abbrev=12 --always` as run
# from DIR.
# The optional MATCH_GLOB_PREFIX argument is passed to the git describe --match option.
# The prefix is chopped off the final version string.
# The optional DIRTY flag enables the git describe --dirty option.
#
function(git_describe DIR OUTPUT)
  cmake_parse_arguments(PARSE_ARGV 0 GIT_ARGS "DIRTY" "MATCH_GLOB_PREFIX" "")
  if(GIT_FOUND)
    if(DEFINED GIT_ARGS_MATCH_GLOB_PREFIX)
      set(MATCH --match "${GIT_ARGS_MATCH_GLOB_PREFIX}*")
    endif()
    if(GIT_ARGS_DIRTY)
      set(DIRTY --dirty)
    else()
      set(DIRTY "")
    endif()
    execute_process(
      COMMAND ${GIT_EXECUTABLE} describe --abbrev=12 --always ${DIRTY} ${MATCH}
      WORKING_DIRECTORY                ${DIR}
      OUTPUT_VARIABLE                  DESCRIPTION
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
      ERROR_VARIABLE                   stderr
      RESULT_VARIABLE                  return_code
    )
    if(return_code)
      message(STATUS "git describe failed: ${stderr}")
    elseif(NOT "${stderr}" STREQUAL "")
      message(STATUS "git describe warned: ${stderr}")
    else()
      if(DEFINED GIT_ARGS_MATCH_GLOB_PREFIX)
        # Remove the version prefix from Git tag.
        string(REPLACE "${GIT_ARGS_MATCH_GLOB_PREFIX}" "" DESCRIPTION ${DESCRIPTION})
      endif()
      # Save output
      set(${OUTPUT} ${DESCRIPTION} PARENT_SCOPE)
    endif()
  endif()
endfunction()
