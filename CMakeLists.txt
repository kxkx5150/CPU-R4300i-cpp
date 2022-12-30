cmake_minimum_required(VERSION 3.11)
project(n64)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exe)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_DEFAULT_SOURCE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-common")

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -DNDEBUG -s -fmerge-all-constants")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -ffast-math -DNDEBUG -s -fmerge-all-constants")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -ffast-math -DNDEBUG -g -fmerge-all-constants")

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")


set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
find_package(OpenAL REQUIRED)
find_package(X11 REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src/utils/unix/x86_64)
set(VIDEO_LIB ${X11_X11_LIB})


if (GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
set(GCC_FLTO_FLAGS "-flto -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${GCC_FLTO_FLAGS}")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} ${GCC_FLTO_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS_MINSIZEREL "${CMAKE_EXE_LINKER_FLAGS_MINSIZEREL} -Wl,--gc-sections")
endif (GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)

if (GCC_VERSION VERSION_GREATER 4.8 OR GCC_VERSION VERSION_EQUAL 4.8)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funsafe-loop-optimizations")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -funsafe-loop-optimizations")
endif (GCC_VERSION VERSION_GREATER 4.8 OR GCC_VERSION VERSION_EQUAL 4.8)

if (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fsanitize=undefined")
endif (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)

if (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffat-lto-objects")
endif (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)

if (GCC_VERSION VERSION_GREATER 5.1 OR GCC_VERSION VERSION_EQUAL 5.1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=discarded-qualifiers")
endif (GCC_VERSION VERSION_GREATER 5.1 OR GCC_VERSION VERSION_EQUAL 5.1)


include_directories(${PROJECT_BINARY_DIR}/src)
include_directories(${PROJECT_BINARY_DIR}/src/utils)
include_directories(${PROJECT_SOURCE_DIR}/src/core)
include_directories(${PROJECT_SOURCE_DIR}/src/arch/x86_64)
include_directories(${PROJECT_SOURCE_DIR}/src/utils/common)
include_directories(${PROJECT_SOURCE_DIR}/src/utils/x11)

include_directories(${ICONV_INCLUDE_DIR})
include_directories(${OPENAL_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})


set(OS_SOURCES
  ${PROJECT_SOURCE_DIR}/src/utils/common/gl_hints.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/input.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/local_time.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/cart_db.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/sha1.cpp
)
set(OS_POSIX_SOURCES
  ${PROJECT_SOURCE_DIR}/src/utils/common/alloc.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/rom_file.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/save_file.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/common/timer.cpp
)
set(OS_X11_SOURCES
  ${PROJECT_SOURCE_DIR}/src/utils/x11/gl_config.cpp
  ${PROJECT_SOURCE_DIR}/src/utils/x11/gl_window.cpp
)

add_executable(n64
  ${EXTRA_OS_EXE}
  ${ASM_SOURCES}
  ${OS_SOURCES}
  ${OS_POSIX_SOURCES}
  ${OS_X11_SOURCES}

  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PROJECT_SOURCE_DIR}/src/core/ai.cpp
  ${PROJECT_SOURCE_DIR}/src/core/bus.cpp
  ${PROJECT_SOURCE_DIR}/src/core/dd.cpp
  ${PROJECT_SOURCE_DIR}/src/core/device.cpp
  ${PROJECT_SOURCE_DIR}/src/core/pi.cpp
  ${PROJECT_SOURCE_DIR}/src/core/rdp.cpp
  ${PROJECT_SOURCE_DIR}/src/core/ri.cpp
  ${PROJECT_SOURCE_DIR}/src/core/rsp.cpp
  ${PROJECT_SOURCE_DIR}/src/core/si.cpp
  ${PROJECT_SOURCE_DIR}/src/core/vi.cpp
  ${PROJECT_SOURCE_DIR}/src/core/cpu.cpp
)
target_link_libraries(n64
	${EXTRA_OS_LIBS}
  ${OPENAL_LIBRARY}
  ${OPENGL_gl_LIBRARY}
  ${ICONV_LIBRARIES}
  ${VIDEO_LIB}
  ${CMAKE_THREAD_LIBS_INIT}
)

