# Minimal stub for FindPythonInterp to satisfy catkin (ROS Noetic)
# without installing the full FindPythonInterp module.
#
# This stub simply sets PYTHON_EXECUTABLE and PYTHONINTERP_VERSION based on the system’s python3.

if(NOT PYTHON_EXECUTABLE)
  # Force use of the system Python (adjust the path if needed)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE STRING "Path to python3 interpreter" FORCE)
endif()

# Retrieve the Python version by executing the interpreter.
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c "import sys; print('.'.join(map(str, sys.version_info[:3])))"
  OUTPUT_VARIABLE _python_version
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(PYTHONINTERP_VERSION "${_python_version}" CACHE STRING "Detected Python interpreter version" FORCE)
set(PYTHONINTERP_FOUND TRUE)
mark_as_advanced(PYTHON_EXECUTABLE PYTHONINTERP_VERSION)
