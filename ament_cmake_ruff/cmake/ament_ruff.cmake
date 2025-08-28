# Copyright 2025 Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Southwest Research Institute® (SwRI®) nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# 

#
# Add a test to check the code for compliance with ruff.
#
# :param TESTNAME: the name of the test, default: "ruff" :type TESTNAME: string
# :param ARGN: the files or directories to check :type ARGN: list of strings
#
# @public
#

function(ament_ruff)
  cmake_parse_arguments(ARG "" "TESTNAME;CONFIG_FILE" "" ${ARGN})
  if(NOT ARG_TESTNAME)
    set(ARG_TESTNAME "ruff")
  endif()

  find_program(ament_ruff_BIN NAMES "ament_ruff")
  if(NOT ament_ruff_BIN)
    message(
      FATAL_ERROR
      "ament_ruff() variable 'ament_ruff_BIN' must not be empty"
    )
  endif()

  set(
    result_file
    "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${ARG_TESTNAME}.xunit.xml"
  )
  set(
    cmd
    "${ament_ruff_BIN}"
    "--xunit-file"
    "${result_file}"
  )
  list(APPEND cmd ${ARG_UNPARSED_ARGUMENTS})
  if(ARG_CONFIG_FILE)
    list(
      APPEND
      cmd
      "--config"
      "${ARG_CONFIG_FILE}"
    )
  elseif(DEFINED ament_cmake_ruff_CONFIG_FILE)
    list(
      APPEND
      cmd
      "--config"
      "${ament_cmake_ruff_CONFIG_FILE}"
    )
  endif()
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_ruff")
  ament_add_test(
      "${ARG_TESTNAME}"
      COMMAND
      ${cmd}
      OUTPUT_FILE
      "${CMAKE_BINARY_DIR}/ament_ruff/${ARG_TESTNAME}.txt"
      RESULT_FILE
      "${result_file}"
      WORKING_DIRECTORY
      "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  set_tests_properties(
    "${ARG_TESTNAME}"
    PROPERTIES
      LABELS
        "ruff;linter"
  )
endfunction()
