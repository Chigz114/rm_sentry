// 感知前端第 2+3 层：ROG-Map 驱动 + 2.5D 投影
//
// 本 node 做两件事：
//
// 【第 2 层】把 ROG-Map 启起来（订阅 /cloud_registered + /Odometry，发布
//           /rog_map/{occ,inf_occ,unk,inf_unk,esdf,map_bound}）。
//
// 【第 3 层】在 ROG-Map 的 C++ API 之上，做 3D → 2.5D 投影：
//           在 world 系下以机器人为中心取 N×N 的 2D 网格，对每个 cell 的 z 柱
//           [robot_z + z_low, robot_z + z_high] 做扫描，任意高度发现障碍则标记 cell
//           为 OCCUPIED，否则全部 Known Free → FREE，有 Unknown → UNKNOWN。
//           输出 nav_msgs/OccupancyGrid 到 /perception/costmap_2d。
//
// 为什么不在这里做距离反平方归一化：
//   ROG-Map 内部已用 log-odds 概率更新 + raycasting miss 清除，每个 voxel 的
//   "是否占据" 已经跨帧累积 + 距离无关化。第 3 层不再重复做归一化，
//   直接对 ROG-Map 的二值输出做空间降维即可。

#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros2.hpp>

using namespace std::chrono_literals;
using super_utils::GridType;

// ----------------------------------------------------------------------------
// SentryROGMap: 继承 ROGMapROS，加一个 2.5D 投影发布器
// ----------------------------------------------------------------------------
class SentryROGMap : public rog_map::ROGMapROS {
public:
    SentryROGMap(const rclcpp::Node::SharedPtr& nh, const std::string& cfg_path)
        : rog_map::ROGMapROS(nh, cfg_path), sentry_nh_(nh) {
        // ---------- 读 2.5D 投影参数 ----------
        sentry_nh_->declare_parameter<double>("costmap2d.resolution",  0.1);
        sentry_nh_->declare_parameter<double>("costmap2d.width_m",    10.0);
        sentry_nh_->declare_parameter<double>("costmap2d.height_m",   10.0);
        sentry_nh_->declare_parameter<double>("costmap2d.z_low",     -0.1);   // 从 robot_z 向下的起点
        sentry_nh_->declare_parameter<double>("costmap2d.z_high",     0.7);   // 从 robot_z 向上的终点（哨兵身高）
        sentry_nh_->declare_parameter<double>("costmap2d.z_step",     0.1);
        sentry_nh_->declare_parameter<double>("costmap2d.rate_hz",    2.0);
        sentry_nh_->declare_parameter<double>("costmap2d.x_offset",   0.0);
        sentry_nh_->declare_parameter<double>("costmap2d.y_offset",   0.0);
        sentry_nh_->declare_parameter<double>("costmap2d.z_offset",   0.0);
        sentry_nh_->declare_parameter<bool>  ("costmap2d.use_inflate", false);
        sentry_nh_->declare_parameter<double>("costmap2d.ground_z", 0.0);       // 地面在地图系的固定 z；z_low/z_high 为相对地面的高度
        sentry_nh_->declare_parameter<bool>  ("costmap2d.follow_robot", true);  // false: costmap 固定在 (x_offset,y_offset)，覆盖全场
        sentry_nh_->declare_parameter<std::string>(
            "costmap2d.topic", "/perception/costmap_2d");
        sentry_nh_->declare_parameter<std::string>(
            "costmap2d.frame_id", "world");

        cm2d_.resolution  = sentry_nh_->get_parameter("costmap2d.resolution").as_double();
        cm2d_.width_m     = sentry_nh_->get_parameter("costmap2d.width_m").as_double();
        cm2d_.height_m    = sentry_nh_->get_parameter("costmap2d.height_m").as_double();
        cm2d_.z_low       = sentry_nh_->get_parameter("costmap2d.z_low").as_double();
        cm2d_.z_high      = sentry_nh_->get_parameter("costmap2d.z_high").as_double();
        cm2d_.z_step      = sentry_nh_->get_parameter("costmap2d.z_step").as_double();
        cm2d_.x_offset    = sentry_nh_->get_parameter("costmap2d.x_offset").as_double();
        cm2d_.y_offset    = sentry_nh_->get_parameter("costmap2d.y_offset").as_double();
        cm2d_.z_offset    = sentry_nh_->get_parameter("costmap2d.z_offset").as_double();
        cm2d_.use_inflate  = sentry_nh_->get_parameter("costmap2d.use_inflate").as_bool();
        cm2d_.ground_z     = sentry_nh_->get_parameter("costmap2d.ground_z").as_double();
        cm2d_.follow_robot = sentry_nh_->get_parameter("costmap2d.follow_robot").as_bool();
        const double rate = sentry_nh_->get_parameter("costmap2d.rate_hz").as_double();
        const std::string topic = sentry_nh_->get_parameter("costmap2d.topic").as_string();
        cm2d_.frame_id = sentry_nh_->get_parameter("costmap2d.frame_id").as_string();

        cm2d_.width  = std::max(1, static_cast<int>(std::round(cm2d_.width_m / cm2d_.resolution)));
        cm2d_.height = std::max(1, static_cast<int>(std::round(cm2d_.height_m / cm2d_.resolution)));

        // ---------- 创建 publisher + timer ----------
        // 用 reliable + volatile，与 RViz / Nav2 默认订阅者兼容
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.reliable().durability_volatile();
        costmap_pub_ = sentry_nh_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic, qos);

        const int period_ms = std::max(1, static_cast<int>(std::round(1000.0 / rate)));
        costmap_cbk_group_ = sentry_nh_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        costmap_timer_ = sentry_nh_->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&SentryROGMap::publishCostmap2D, this),
            costmap_cbk_group_);

        RCLCPP_INFO(sentry_nh_->get_logger(),
                    "2.5D 投影启用: %dx%d cells @ %.2fm, center=(%.2f, %.2f) %s, "
                    "扫描 z∈[%.2f, %.2f] (ground_z=%.3f + 离地[%.2f,%.2f]), "
                    "频率 %.1f Hz, 膨胀=%s, topic=%s",
                    cm2d_.width, cm2d_.height, cm2d_.resolution,
                    cm2d_.x_offset, cm2d_.y_offset,
                    cm2d_.follow_robot ? "(跟随机器人)" : "(固定全场)",
                    cm2d_.ground_z + cm2d_.z_low, cm2d_.ground_z + cm2d_.z_high,
                    cm2d_.ground_z, cm2d_.z_low, cm2d_.z_high,
                    rate,
                    cm2d_.use_inflate ? "true" : "false",
                    topic.c_str());
    }

private:
    struct Costmap2DCfg {
        double resolution;
        double width_m, height_m;
        int    width, height;     // cells
        std::string frame_id{"world"};
        double z_low, z_high, z_step;
        double x_offset, y_offset, z_offset;
        bool   use_inflate;
        double ground_z;
        bool   follow_robot;
    } cm2d_;

    rclcpp::Node::SharedPtr sentry_nh_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr costmap_timer_;
    rclcpp::CallbackGroup::SharedPtr costmap_cbk_group_;

    // ------------------------------------------------------------------------
    // publishCostmap2D: 主 2.5D 投影逻辑
    // ------------------------------------------------------------------------
    void publishCostmap2D() {
        // 没收到 odom 前 robot_state_ 无效，skip
        if (!robot_state_.rcv) {
            return;
        }

        const rog_map::Vec3f robot_p = robot_state_.p;
        // follow_robot=true: 以机器人为中心的滑动窗口；false: 固定在 (x_offset,y_offset)，覆盖全场
        const double center_x = cm2d_.follow_robot ? (robot_p.x() + cm2d_.x_offset) : cm2d_.x_offset;
        const double center_y = cm2d_.follow_robot ? (robot_p.y() + cm2d_.y_offset) : cm2d_.y_offset;
        const rog_map::Vec3f local_center(center_x, center_y, robot_p.z() + cm2d_.z_offset);

        // costmap 以带偏移的局部框中心为中心
        const double origin_x = local_center.x() - cm2d_.width  * cm2d_.resolution / 2.0;
        const double origin_y = local_center.y() - cm2d_.height * cm2d_.resolution / 2.0;

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = cm2d_.frame_id;
        grid.header.stamp = sentry_nh_->get_clock()->now();
        grid.info.resolution = static_cast<float>(cm2d_.resolution);
        grid.info.width  = cm2d_.width;
        grid.info.height = cm2d_.height;
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(cm2d_.width * cm2d_.height, -1);  // 默认 UNKNOWN

        // z 扫描层数
        const int z_steps = std::max(
            1, static_cast<int>(std::ceil((cm2d_.z_high - cm2d_.z_low) / cm2d_.z_step)));

        // 对每个 xy cell 扫一列 z
        for (int j = 0; j < cm2d_.height; ++j) {
            for (int i = 0; i < cm2d_.width; ++i) {
                const double x = origin_x + (i + 0.5) * cm2d_.resolution;
                const double y = origin_y + (j + 0.5) * cm2d_.resolution;

                bool any_unknown = false;
                bool any_known_free = false;
                int8_t cell_val = -1;

                for (int k = 0; k < z_steps; ++k) {
                    // 地面在地图系固定于 cm2d_.ground_z（实测值）
                    // z_low/z_high 含义：离地高度(m)，不依赖 robot_z，机器人上高地时参考不漂
                    const double z = cm2d_.ground_z + cm2d_.z_low + (k + 0.5) * cm2d_.z_step;
                    const rog_map::Vec3f pos(x, y, z);

                    // use_inflate=true 用膨胀图（给 planner 用）
                    // use_inflate=false 用原始概率图（给自己再次膨胀时）
                    GridType gt = cm2d_.use_inflate
                                  ? inf_map_->getGridType(pos)
                                  : getGridType(pos);

                    if (gt == super_utils::OCCUPIED) {
                        cell_val = 100;  // OCCUPIED
                        break;
                    } else if (gt == super_utils::KNOWN_FREE) {
                        any_known_free = true;
                    } else if (gt == super_utils::UNKNOWN) {
                        any_unknown = true;
                    }
                }

                const int idx = j * cm2d_.width + i;
                if (cell_val == 100) {
                    grid.data[idx] = 100;
                } else if (any_known_free) {
                    // 有已探测的 free 层 → 这个 xy 柱可通行
                    grid.data[idx] = 0;
                } else if (any_unknown) {
                    grid.data[idx] = -1;
                } else {
                    // 所有层都不在 local map 范围内 / 或被 virtual ceil/ground 边界外判 OCCUPIED
                    // 保守起见标 UNKNOWN
                    grid.data[idx] = -1;
                }
            }
        }

        costmap_pub_->publish(grid);
    }
};

// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("perception_mapper");

    node->declare_parameter<std::string>("cfg_path", "");
    std::string cfg_path = node->get_parameter("cfg_path").as_string();

    if (cfg_path.empty()) {
        RCLCPP_ERROR(node->get_logger(),
                     "cfg_path parameter is empty. 请用 ros2 run 或 launch 指定 -p cfg_path:=<abs_path>");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "perception_mapper 启动，读取配置: %s", cfg_path.c_str());

    // 构造 SentryROGMap（继承 ROGMapROS，加 2.5D 投影）
    auto sentry_rog = std::make_shared<SentryROGMap>(node, cfg_path);
    (void)sentry_rog;  // 持有引用防止析构

    // MultiThreadedExecutor 让 ROG-Map 的多个 callback groups 可并行，
    // 以及新加的 costmap2d timer 和其它 callback 独立跑
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
