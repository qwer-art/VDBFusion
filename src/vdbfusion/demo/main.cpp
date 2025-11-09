#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <vdbfusion/MarchingCubesConst.h>
#include <vdbfusion/VDBVolume.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

using namespace Eigen;

void DebugCloud() {
    // 初始化OpenVDB库
    openvdb::initialize();

    // 1. 定义原始点（传感器原点）
    Eigen::Vector3d origin(0.0f, 0.0f, 5.0f);  // 传感器位置
    std::cout << "传感器原点: (" << origin.x() << ", " << origin.y() << ", " << origin.z() << ")"
              << std::endl;

    // 2. 创建模拟LiDAR点云数据（实际应用中应从文件或传感器读取）
    std::vector<Eigen::Vector3d> point_cloud;

    // 生成一个简单的立方体点云作为测试数据
    const float cube_size = 2.0f;
    const float point_spacing = 0.2f;

    // 生成立方体六个面的点
    for (float x = -cube_size; x <= cube_size; x += point_spacing) {
        for (float y = -cube_size; y <= cube_size; y += point_spacing) {
            // 前面和后面
            point_cloud.emplace_back(x, y, -cube_size);
            point_cloud.emplace_back(x, y, cube_size);
            // 左面和右面
            point_cloud.emplace_back(-cube_size, x, y);
            point_cloud.emplace_back(cube_size, x, y);
            // 底面和顶面
            point_cloud.emplace_back(x, -cube_size, y);
            point_cloud.emplace_back(x, cube_size, y);
        }
    }

    std::cout << "生成点云数量: " << point_cloud.size() << " 个点" << std::endl;

    // 3. 配置并创建VDBVolume
    float voxel_size = 0.1f;     // 体素大小（米）
    float sdf_trunc = 0.3f;      // 截断距离
    bool space_carving = false;  // 是否启用空间雕刻

    auto volume = std::make_unique<vdbfusion::VDBVolume>(voxel_size, sdf_trunc, space_carving);
    std::cout << "VDBVolume配置 - 体素大小: " << voxel_size << ", 截断距离: " << sdf_trunc
              << std::endl;

    // 4. 将点云积分到VDBVolume中（使用单位变换矩阵）
    auto gaussianWeight = [](float distance) -> float {
        // float sigma = 0.1f; // 控制衰减速度的参数，标准差越小，权重衰减越快
        // return std::exp(-(distance * distance) / (2 * sigma * sigma));
        return 2.f;
    };
    volume->Integrate(point_cloud, origin, gaussianWeight);
    std::cout << "点云积分完成" << std::endl;

    // 5. 提取三角形网格（顶点和三角形）
    bool fill_holes{true};
    float min_weight{2.0};
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> mesh =
        volume->ExtractTriangleMesh(fill_holes, min_weight);
    std::vector<Eigen::Vector3d> const& vertices{std::get<0>(mesh)};
    std::vector<Eigen::Vector3i> const& triangles{std::get<1>(mesh)};
    std::cout << "vertices: " << vertices.size() << ",triangles: " << triangles.size() << std::endl;

    // 清理资源
    openvdb::uninitialize();
    std::cout << "\n处理完成!" << std::endl;
}

void DebugPoint() {
    // 初始化OpenVDB库
    openvdb::initialize();

    // 1. 定义原始点（传感器原点）
    Eigen::Vector3d origin(0.0f, 0.0f, 0.0f);  // 传感器位置
    std::cout << "传感器原点: (" << origin.x() << ", " << origin.y() << ", " << origin.z() << ")"
              << std::endl;

    // 2. 创建模拟LiDAR点云数据（实际应用中应从文件或传感器读取）
    std::vector<Eigen::Vector3d> point_cloud;
    Eigen::Vector3d point1{5.f, 0.f, 0.f};
    Eigen::Vector3d point2{5.f, 0.2f, 0.f};
    Eigen::Vector3d point3{5.f, 0.f, 0.2f};
    Eigen::Vector3d point4{5.f, 0.1f, 0.f};
    Eigen::Vector3d point5{5.f, 0.f, 0.1f};
    point_cloud.emplace_back(point1);
    point_cloud.emplace_back(point2);
    point_cloud.emplace_back(point3);

    // 生成一个简单的立方体点云作为测试数据
    const float cube_size = 2.0f;
    const float point_spacing = 0.2f;
    std::cout << "生成点云数量: " << point_cloud.size() << " 个点" << std::endl;

    // 3. 配置并创建VDBVolume
    float voxel_size = 0.1f;     // 体素大小（米）
    float sdf_trunc = 0.3f;      // 截断距离
    bool space_carving = false;  // 是否启用空间雕刻

    auto volume = std::make_unique<vdbfusion::VDBVolume>(voxel_size, sdf_trunc, space_carving);
    std::cout << "VDBVolume配置 - 体素大小: " << voxel_size << ", 截断距离: " << sdf_trunc
              << std::endl;

    // 4. 将点云积分到VDBVolume中（使用单位变换矩阵）
    auto gaussianWeight = [](float distance) -> float {
        float sigma = 0.1f;
        return std::exp(-(distance * distance) / (2 * sigma * sigma));
    };
    volume->Debug(point_cloud, origin, gaussianWeight);
    // volume->Integrate(point_cloud,origin,gaussianWeight);
    std::cout << "点云积分完成" << std::endl;

    // 5. 提取三角形网格（顶点和三角形）
    bool fill_holes{false};
    float min_weight{1.0};
    // std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> mesh =
    // volume->ExtractTriangleMesh(fill_holes, min_weight);
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> mesh =
        volume->DebugExtract(fill_holes, min_weight);
    std::vector<Eigen::Vector3d> const& vertices{std::get<0>(mesh)};
    std::vector<Eigen::Vector3i> const& triangles{std::get<1>(mesh)};
    std::cout << "vertices: " << vertices.size() << ",triangles: " << triangles.size() << std::endl;

    // // 清理资源
    // openvdb::uninitialize();
    std::cout << "\n处理完成!" << std::endl;
}

float ComputeSDF(const Eigen::Vector3d& origin,
                 const Eigen::Vector3d& point,
                 const Eigen::Vector3d& voxel_center) {
    const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
    const Eigen::Vector3d v_point_voxel = point - voxel_center;
    const double dist = v_point_voxel.norm();
    const double proj = v_voxel_origin.dot(v_point_voxel);
    const double sign = proj / std::abs(proj);
    return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord& voxel, const openvdb::math::Transform& xform) {
    const float voxel_size = xform.voxelSize()[0];
    openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
    return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

namespace debug {
static const openvdb::Coord shift[8] = {
    openvdb::Coord(0, 0, 0), openvdb::Coord(1, 0, 0), openvdb::Coord(1, 1, 0),
    openvdb::Coord(0, 1, 0), openvdb::Coord(0, 0, 1), openvdb::Coord(1, 0, 1),
    openvdb::Coord(1, 1, 1), openvdb::Coord(0, 1, 1),
};

}

void DebugNodes() {
    // for(int32_t cube_index{0};cube_index < 256;++cube_index){
    //     std::bitset<8> node_bits(cube_index);
    //     int edge{edge_table[cube_index]};
    //     std::bitset<12> edge_bits(edge);
    //     std::cout << "cube: " << cube_index << ",node: " << node_bits << ",edge: " << edge_bits
    //     << std::endl;
    // }
    // int32_t cube_index{1};
    // std::bitset<8> node_bits(cube_index);
    // int edge{edge_table[cube_index]};
    // std::bitset<12> edge_bits(edge);
    // std::cout << "Node: " << cube_index << ",node_bits: " << node_bits << ",edge_bits: " <<
    // edge_bits << std::endl;
    Vector3d origin{0., 0., 0.};
    Vector3d point{5., 0., 0.};
    Vector3d direction{point - origin};

    //// grid
    float voxel_size = 0.1f;     // 体素大小（米）
    float sdf_trunc = 0.3f;      // 截断距离
    bool space_carving = false;  // 是否启用空间雕刻
    openvdb::initialize();
    openvdb::FloatGrid::Ptr tsdf_;
    openvdb::FloatGrid::Ptr weights_;
    tsdf_ = openvdb::FloatGrid::create(sdf_trunc);
    tsdf_->setName("D(x): signed distance grid");
    tsdf_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size));
    tsdf_->setGridClass(openvdb::GRID_LEVEL_SET);
    weights_ = openvdb::FloatGrid::create(0.0f);
    weights_->setName("W(x): weights grid");
    weights_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size));
    weights_->setGridClass(openvdb::GRID_UNKNOWN);

    auto gaussianWeight = [](float distance) -> float {
        float sigma = 0.1f;
        return std::exp(-(distance * distance) / (2 * sigma * sigma));
    };

    /// 1. 处理每个point
    openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
    dir.normalize();
    float depth = static_cast<float>(direction.norm());
    float t0 = depth - sdf_trunc;
    float t1 = depth + sdf_trunc;
    auto ray = openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf_);
    openvdb::Vec3R start{ray.start()};
    openvdb::Vec3R end{ray.end()};
    std::cout << "[Point],dir: " << direction.transpose() << std::endl;
    printf("[Point],start: (%.2f,%.2f,%.2f),end: (%.2f,%.2f,%.2f)\n", start.x(), start.y(),
           start.z(), end.x(), end.y(), end.z());
    openvdb::math::DDA<decltype(ray)> dda(ray);
    int32_t p2v_count{0};
    auto tsdf_acc = tsdf_->getUnsafeAccessor();
    auto weights_acc = weights_->getUnsafeAccessor();
    do {
        openvdb::math::Coord const& voxel = dda.voxel();
        Vector3d voxel_center = GetVoxelCenter(voxel, tsdf_->transform());
        const auto sdf = ComputeSDF(origin, point, voxel_center);
        if (sdf > -sdf_trunc) {
            const float tsdf = std::min(sdf_trunc, sdf);
            const float weight = gaussianWeight(sdf);
            const float last_weight = weights_acc.getValue(voxel);
            const float last_tsdf = tsdf_acc.getValue(voxel);
            const float new_weight = weight + last_weight;
            const float new_tsdf = (last_tsdf * last_weight + tsdf * weight) / (new_weight);
            printf(
                "[Point2Voxel],voxel: (%d,%d,%d),voxel_center: (%.2f,%.2f,%.2f),tsdf: %.3f,weight: %.3f,last_weight: %.3f,last_tsdf: %.3f,new_weight: "
                "%.3f,new_tsdf: %.3f\n",
                voxel.x(),voxel.y(),voxel.z(),voxel_center.x(),voxel_center.y(),voxel_center.z(),
                tsdf, weight, last_weight, last_tsdf, new_weight, new_tsdf);
            tsdf_acc.setValue(voxel, new_tsdf);
            weights_acc.setValue(voxel, new_weight);
        }
    } while (dda.step());

    /// 2. 处理每个voxel
    int32_t voxel_count{0};
    for (auto iter = tsdf_->beginValueOn(); iter; ++iter) {
        const openvdb::Coord& voxel = iter.getCoord();
        const int32_t x = voxel.x();
        const int32_t y = voxel.y();
        const int32_t z = voxel.z();
        printf("[Voxel],index: (%d),idx: (%d,%d,%d)\n", voxel_count++, x, y, z);
        for (int i = 0; i < 8; i++) {
            openvdb::Coord idx = voxel + debug::shift[i];
            float node_weight{weights_acc.getValue(idx)};
            float node_tsdf{tsdf_acc.getValue(idx)};
            printf("[Voxel2Node],node_weight: (%.3f),node_tsdf: (%.3f),node_idx: (%d,%d,%d)\n",node_weight,node_tsdf,idx.x(),idx.y(),idx.z());
        }

    }
}

int main() {
    // DebugCloud();
    // DebugPoint();
    DebugNodes();
    return 0;
}