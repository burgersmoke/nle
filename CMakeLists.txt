cmake_minimum_required(VERSION 3.15)
file(STRINGS "version.txt" NLE_VERSION)
project(nle VERSION ${NLE_VERSION})

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
  # Unclear if this is even necessary. `dsymutil rlmain -o rlmain.dSYM` seems to
  # have done the trick.
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
  set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf-with-dsym")

  if(0)
    # address sanitizer.
    set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_C_FLAGS_DEBUG
        "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_LINKER_FLAGS_DEBUG
        "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address"
    )
  endif()
elseif(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
else()
  message("Some other build type.")
endif()

message(STATUS "Building nle backend version: ${NLE_VERSION}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# We use this to decide where the root of the nle/ package is. Normally it
# shouldn't be needed, but sometimes (e.g. when using setuptools) we are
# generating some of the files outside of the original package path.
set(PYTHON_SRC_PARENT
    ${nle_SOURCE_DIR}
    CACHE STRING "Directory containing the nle package files")

set(HACKDIR
    "$ENV{HOME}/nethackdir.nle"
    CACHE STRING "Configuration files for nethack")

message(STATUS "HACKDIR set to: ${HACKDIR}")

# Playground vars
set(VARDIR ${HACKDIR})
set(INSTDIR ${HACKDIR})

add_compile_definitions(
  GCC_WARN
  NOCLIPPING
  NOMAIL
  NOTPARMDECL
  HACKDIR="${HACKDIR}"
  DEFAULT_WINDOW_SYS="rl"
  DLB)

set(NLE_SRC ${nle_SOURCE_DIR}/src)
set(NLE_INC ${nle_SOURCE_DIR}/include)
set(NLE_DAT ${nle_SOURCE_DIR}/dat)
set(NLE_UTIL ${nle_SOURCE_DIR}/util)
set(NLE_DOC ${nle_SOURCE_DIR}/doc)
# Some Windows files are located only here
set(NLE_SYS_WINNT ${nle_SOURCE_DIR}/sys/winnt)

set(NLE_SRC_GEN ${nle_BINARY_DIR}/src)
set(NLE_INC_GEN ${nle_BINARY_DIR}/include)
set(NLE_DAT_GEN ${nle_BINARY_DIR}/dat)
set(NLE_UTIL_GEN ${nle_BINARY_DIR}/util)

# EXCLUDE_FROM_ALL: Don't install this static library into /usr/local.
add_subdirectory(third_party/deboost.context EXCLUDE_FROM_ALL)
add_subdirectory(util)
add_subdirectory(dat)

# this is empty except for Windows currently
set(WINDOWS_INC "")

IF (WIN32)
  # windows sources

  # prevent termiWin from redefining core IO
  add_definitions(-DTERMIWIN_DONOTREDEFINE)

  # Windows NetHack definitions
  # See the NetHack 3.6 branch project for more info:
  #https://github.com/NetHack/NetHack/blob/NetHack-3.6/win/win32/vs2017/NetHack.vcxproj
  # <PreprocessorDefinitions>TILES;WIN32CON;DLB;MSWIN_GRAPHICS;SAFEPROCS;_LIB;%(PreprocessorDefinitions)</PreprocessorDefinitions>
  #add_definitions(-DTILES)
  add_definitions(-DWIN32CON)
  add_definitions(-DDLB)

  # Two different graphics defines under Windows
  #add_definitions(-DMSWIN_GRAPHICS)
  add_definitions(-DTTY_GRAPHICS)

  # Allow GUI stubs to link
  add_definitions(-DGUISTUB)

  add_definitions(-DWIN32CON)
  add_definitions(-DSAFEPROCS)
  add_definitions(-D_LIB)

  # add Windows specific dlfcn implementation header
  list(APPEND WINDOWS_INC "${nle_SOURCE_DIR}/third_party/dlfcn-win32/src")
  list(APPEND WINDOWS_INC "${nle_SOURCE_DIR}/third_party/termiWin/include")
  list(APPEND WINDOWS_INC "${nle_SOURCE_DIR}/third_party/deboost.context/include")

  add_subdirectory(${nle_SOURCE_DIR}/third_party/dlfcn-win32)

  # Windows source files.
  # NOTE: Background and decisions on which source files to include came from the NetHack 3.6 VS 2017 project:
  # #https://github.com/NetHack/NetHack/blob/NetHack-3.6/win/win32/vs2017/NetHack.vcxproj
  file(
  GLOB
  NETHACK_SRC
  "src/*.c"
  "sys/share/cppregex.cpp"
  "sys/winnt/windmain.c"
  "sys/winnt/nttty.c"
  "sys/winnt/winnt.c"
  "sys/winnt/stubs.c"
  "sys/winnt/ntsound.c"
  "win/tty/*.c"
  "win/share/safeproc.c"
  "win/rl/winrl.cc"
  "third_party/libtmt/tmt.c"
  )


  # since there are some issues with rlmain, let's also try to set up a vanilla
  # nethack build here for a parity check...
  add_executable(nethackwin ${NETHACK_SRC} )
  target_include_directories(nethackwin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${NLE_INC_GEN} /usr/local/include
                                          ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtmt
                                          ${WINDOWS_INC})
  add_dependencies(nethackwin util dat)
  target_link_libraries(nethackwin PUBLIC fcontext bcrypt Winmm)

ELSE()
  # other systems sources (POSIX)
  file(
  GLOB
  NETHACK_SRC
  "src/*.c"
  "sys/share/posixregex.c"
  "sys/share/ioctl.c"
  "sys/unix/unixunix.c"
  "sys/unix/unixmain.c"
  "sys/unix/unixres.c"
  "win/tty/*.c"
  "win/rl/winrl.cc"
  "third_party/libtmt/tmt.c")
ENDIF()

# libnethack library
add_library(nethack SHARED ${NETHACK_SRC})
add_dependencies(nethack util dat)
set_target_properties(nethack PROPERTIES CXX_STANDARD 14 SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
target_include_directories(nethack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${NLE_INC_GEN} /usr/local/include
                                          ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtmt
                                          ${WINDOWS_INC})

target_link_directories(nethack PUBLIC /usr/local/lib)

# Careful with -DMONITOR_HEAP: Ironically, it fails to fclose FILE* heaplog.
# target_compile_definitions(nethack PUBLIC "$<$<CONFIG:DEBUG>:MONITOR_HEAP>")

IF (WIN32)
    target_link_libraries(nethack PUBLIC fcontext bcrypt Winmm)
ELSE()
    target_link_libraries(nethack PUBLIC m fcontext bz2)
ENDIF()

IF(WIN32)
    # Windows specific DLLs which are needed to be loaded at runtime

    add_library(nhdefkey SHARED "sys/winnt/nhdefkey.c")
    add_dependencies(nhdefkey util)
    target_include_directories(nhdefkey PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${NLE_INC_GEN}
                                          ${WINDOWS_INC})

    add_library(nhraykey SHARED "sys/winnt/nhraykey.c")
    add_dependencies(nhraykey util)
    target_include_directories(nhraykey PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                          ${NLE_INC_GEN}
                                          ${WINDOWS_INC})

ENDIF()

# dlopen wrapper library
add_library(nethackdl STATIC "sys/unix/nledl.c")
target_include_directories(nethackdl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                            ${WINDOWS_INC})
target_link_libraries(nethackdl PUBLIC dl)

IF (WIN32)
    # this includes a DLL which needs to push "up" a level from bin
    install(FILES "${nle_BINARY_DIR}/bin/dl.dll" DESTINATION ${nle_BINARY_DIR})
ENDIF()


# rlmain C++ (test) binary
add_executable(rlmain "sys/unix/rlmain.cc")
set_target_properties(rlmain PROPERTIES CXX_STANDARD 11)
target_link_libraries(rlmain PUBLIC nethackdl)

target_include_directories(rlmain PUBLIC ${NLE_INC_GEN})
add_dependencies(rlmain util) # For pm.h.

# pybind11 python library.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/pybind11)
pybind11_add_module(_pynethack win/rl/pynethack.cc src/monst.c src/decl.c
                    src/drawing.c src/objects.c)
target_link_libraries(_pynethack PUBLIC nethackdl)
set_target_properties(_pynethack PROPERTIES CXX_STANDARD 14)
target_include_directories(_pynethack PUBLIC ${NLE_INC_GEN})
add_dependencies(_pynethack util) # For pm.h.
