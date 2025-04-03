cmake_minimum_required(VERSION 3.10)

# GoogleTestを見つける
find_package(GTest REQUIRED)

# テスト実行ファイルを作成
add_executable(distance_table_test distance_table_test.cpp)

# インクルードディレクトリの設定
target_include_directories(distance_table_test
    PRIVATE
        ${CMAKE_SOURCE_DIR}/inc
        ${CMAKE_SOURCE_DIR}/src
)

# 必要なライブラリをリンク
target_link_libraries(distance_table_test
    PRIVATE
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}_lib
)

# テストを登録
add_test(NAME distance_table_test COMMAND distance_table_test) 