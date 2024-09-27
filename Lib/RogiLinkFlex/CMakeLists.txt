cmake_minimum_required(VERSION 3.10)
project(RogiLinkFlex CXX)  # プロジェクトをC++として設定

# C++の標準を設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# HALライブラリのインクルードディレクトリ
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32xxxx_HAL_Driver/Inc
)

# ソースファイルを取得
file(GLOB_RECURSE SOURCES "Src/*.cpp")  # C++のソースファイルを取得

# ライブラリの作成
add_library(RogiLinkFlex STATIC ${SOURCES})
