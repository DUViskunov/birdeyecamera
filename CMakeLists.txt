cmake_minimum_required(VERSION 3.16)
project(workzone_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(Threads REQUIRED)

# Ядро системы. Внешних зависимостей нет: stb лежит в third_party,
# математика калибровки и склейки реализована здесь же.
add_library(wz_core STATIC
    src/stb_impl.cpp
    src/image.cpp
    src/geometry.cpp
    src/calibration.cpp
    src/synthetic.cpp
    src/source.cpp
    src/source_camera_win.cpp
    src/sync.cpp
    src/stitcher.cpp
    src/chessboard.cpp
    src/intrinsics_calib.cpp
    src/json.cpp
    src/config.cpp
    src/pipeline.cpp
    src/http_server.cpp
)

target_include_directories(wz_core PUBLIC include third_party)
target_link_libraries(wz_core PUBLIC Threads::Threads)

if(WIN32)
    # ws2_32 - сокеты сервера; остальное - Media Foundation для захвата с камер.
    target_link_libraries(wz_core PUBLIC ws2_32 mf mfplat mfreadwrite mfuuid ole32 oleaut32 uuid)
endif()

if(MSVC)
    target_compile_options(wz_core PRIVATE /W4)
else()
    # stb собирается с чужими предупреждениями, поэтому -Werror не включаем.
    target_compile_options(wz_core PRIVATE -Wall -Wextra)
    set_source_files_properties(src/stb_impl.cpp PROPERTIES COMPILE_OPTIONS "-w")
endif()

add_executable(workzone_server apps/workzone_server.cpp)
target_link_libraries(workzone_server PRIVATE wz_core)

add_executable(gen_testdata apps/gen_testdata.cpp)
target_link_libraries(gen_testdata PRIVATE wz_core)

add_executable(calibrate apps/calibrate.cpp)
target_link_libraries(calibrate PRIVATE wz_core)

add_executable(calibrate_intrinsics apps/calibrate_intrinsics.cpp)
target_link_libraries(calibrate_intrinsics PRIVATE wz_core)

# Тесты: никакого фреймворка, обычные исполняемые файлы с кодом возврата.
enable_testing()

foreach(test_name geometry sync stitch intrinsics)
    add_executable(test_${test_name} tests/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} PRIVATE wz_core)
    add_test(NAME ${test_name} COMMAND test_${test_name})
endforeach()
