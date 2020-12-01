# unsets all public (designed to be used externally) variables and reports
macro(Scanner_REPORT_NOT_FOUND REASON_MSG)
    unset(Scanner_FOUND)
    unset(Scanner_INCLUDE_DIRS)
    unset(Scanner_LIBRARIES)
    unset(Scanner_WORLD_VERSION)
    unset(Scanner_MAJOR_VERSION)
    unset(Scanner_MINOR_VERSION)
    # Make results of search visible in the CMake GUI if Scanner has not
    # been found so that user does not have to toggle to advanced view.
    mark_as_advanced(CLEAR Scanner_INCLUDE_DIR)
    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(Scanner_FIND_QUIETLY)
        message(STATUS "Failed to find Scanner - " ${REASON_MSG} ${ARGN})
    elseif(Scanner_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find Scanner - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find Scanner - " ${REASON_MSG} ${ARGN})
    endif()
endmacro(Scanner_REPORT_NOT_FOUND)

# Search user-installed locations first, so that we prefer user installs
# to system installs where both exist.
list(APPEND Scanner_CHECK_INCLUDE_DIRS
    $ENV{HXI_SCANNER_INCLUDE_PATH})
list(APPEND Scanner_CHECK_LIBRARY_DIRS
    $ENV{HXI_SCANNER_BINARY_PATH})         

message(STATUS "Scanner Scanner_CHECK_INCLUDE_DIRS: " ${Scanner_CHECK_INCLUDE_DIRS})
message(STATUS "Scanner Scanner_CHECK_LIBRARY_DIRS: " ${Scanner_CHECK_LIBRARY_DIRS})

# Find include directory for Scanner
find_path(Scanner_INCLUDE_DIR
    NAMES HxiDefinitions.h
    PATHS ${Scanner_CHECK_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
if(NOT Scanner_INCLUDE_DIR OR NOT EXISTS ${Scanner_INCLUDE_DIR})
    Scanner_REPORT_NOT_FOUND(
        "Could not find Scanner include directory")
else()
    message(STATUS "Scanner include dir found: " ${Scanner_INCLUDE_DIR})
endif()

# Find library directory for Scanner
find_library(Scanner_LIBRARY
    NAMES HxiScanner
    PATHS ${Scanner_CHECK_LIBRARY_DIRS}
    NO_DEFAULT_PATH)
if(NOT Scanner_LIBRARY OR NOT EXISTS ${Scanner_LIBRARY})
    Scanner_REPORT_NOT_FOUND(
        "Could not find Scanner library.")
else()
    # TODO: need to fix this hacky solution for getting Scanner_LIBRARY_DIR
    string(REGEX MATCH ".*/" Scanner_LIBRARY_DIR ${Scanner_LIBRARY})
    message(STATUS "Scanner library dir found: " ${Scanner_LIBRARY_DIR})
endif()

# Mark internally as found, then verify. Scanner_REPORT_NOT_FOUND() unsets if
# called.
set(Scanner_FOUND TRUE)

# Set standard CMake FindPackage variables if found.
if(Scanner_FOUND)
    set(Scanner_INCLUDE_DIRS ${Scanner_INCLUDE_DIR})
    file(GLOB Scanner_LIBRARIES ${Scanner_LIBRARY_DIR}HxiScanner.lib)
endif()

# Handle REQUIRED / QUIET optional arguments.
include(FindPackageHandleStandardArgs)
if(Scanner_FOUND)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(Scanner DEFAULT_MSG
    Scanner_INCLUDE_DIRS Scanner_LIBRARIES)
endif()

# Only mark internal variables as advanced if we found Scanner, otherwise
# leave it visible in the standard GUI for the user to set manually.
if(Scanner_FOUND)
    mark_as_advanced(FORCE Scanner_INCLUDE_DIR Scanner_LIBRARY)
endif()
