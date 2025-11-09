# 查找必要的依赖包
find_package(OpenVDB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# 创建测试可执行文件
add_executable(vdbfusion_test main.cpp)

# 链接依赖库
target_link_libraries(vdbfusion_test
    PRIVATE 
    vdbfusion
    OpenVDB::openvdb
    Eigen3::Eigen
    GTest::gtest
    GTest::gtest_main
)

# 添加包含目录
target_include_directories(vdbfusion_test
    PRIVATE 
    ${CMAKE_SOURCE_DIR}/vdbfusion
)

# 启用C++17标准
target_compile_features(vdbfusion_test PRIVATE cxx_std_17)

# 注册测试
enable_testing()
add_test(NAME vdbfusion_unit_tests COMMAND vdbfusion_test)