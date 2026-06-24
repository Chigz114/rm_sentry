#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyxml2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

class TraversabilityMapper : public rclcpp::Node {
public:
    TraversabilityMapper() : Node("traversability_mapper") {
        declareParams();

        grid_w_ = static_cast<int>(std::round(width_m_ / resolution_));
        grid_h_ = static_cast<int>(std::round(height_m_ / resolution_));
        origin_x_ = x_offset_ - grid_w_ * resolution_ / 2.0;
        origin_y_ = y_offset_ - grid_h_ * resolution_ / 2.0;

        log_odds_.assign(grid_w_ * grid_h_, 0.0f);
        hit_counts_.assign(grid_w_ * grid_h_, 0);
        frame_min_z_.assign(grid_w_ * grid_h_, 1e9f);
        static_prior_.assign(grid_w_ * grid_h_, false);

        if (!world_file_.empty()) {
            loadStaticPrior();
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS pub_qos(rclcpp::KeepLast(5));
        pub_qos.reliable().durability_volatile();
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            costmap_topic_, pub_qos);

        rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
        sub_qos.best_effort().durability_volatile();

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, sub_qos,
            std::bind(&TraversabilityMapper::cloudCallback, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, sub_qos,
            std::bind(&TraversabilityMapper::odomCallback, this, std::placeholders::_1));

        const int period_ms = std::max(1, static_cast<int>(std::round(1000.0 / rate_hz_)));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&TraversabilityMapper::publishCostmap, this));

        RCLCPP_INFO(get_logger(),
            "traversability_mapper: %dx%d @ %.2fm, origin=(%.2f,%.2f), "
            "h_climb=%.2f, decay_tau=%.2f, occ_thresh=%.2f, rate=%.1f Hz",
            grid_w_, grid_h_, resolution_, origin_x_, origin_y_,
            h_climb_, decay_tau_, occ_thresh_, rate_hz_);
    }

private:
    // ---- params ----
    double resolution_{0.1};
    double width_m_{13.0}, height_m_{9.0};
    double x_offset_{0.0}, y_offset_{0.0};
    double h_climb_{0.10};
    double ground_clamp_lo_{-0.55}, ground_clamp_hi_{-0.25};
    double ground_init_{-0.40};
    int n_min_near_{4}, n_min_mid_{2}, n_min_far_{1};
    double near_dist_{2.0}, mid_dist_{4.0};
    double delta_hit_{0.4}, log_odds_cap_{2.0}, log_odds_floor_{-2.0};
    double decay_tau_{0.7};
    double occ_thresh_{0.5};
    double rate_hz_{10.0};
    std::string cloud_topic_{"/cloud_registered"};
    std::string odom_topic_{"/Odometry"};
    std::string costmap_topic_{"/perception/costmap_2d"};
    std::string frame_id_{"map"};
    std::string world_file_;
    std::string odom_frame_{"lidar_odom"};

    // ---- grid ----
    int grid_w_{0}, grid_h_{0};
    double origin_x_{0}, origin_y_{0};
    std::vector<float> log_odds_;
    std::vector<int> hit_counts_;
    std::vector<float> frame_min_z_;
    std::vector<bool> static_prior_;
    float global_ground_z_{-0.4f};

    // ---- state ----
    bool have_odom_{false};
    bool tf_ready_{false};
    double robot_x_{0}, robot_y_{0};
    rclcpp::Time last_decay_time_;
    std::chrono::steady_clock::time_point last_decay_wall_;

    // ---- TF ----
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ---- ROS interfaces ----
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- ROS ----

    void declareParams() {
        declare_parameter<double>("resolution", resolution_);
        declare_parameter<double>("width_m", width_m_);
        declare_parameter<double>("height_m", height_m_);
        declare_parameter<double>("x_offset", x_offset_);
        declare_parameter<double>("y_offset", y_offset_);
        declare_parameter<double>("h_climb", h_climb_);
        declare_parameter<double>("ground_clamp_lo", ground_clamp_lo_);
        declare_parameter<double>("ground_clamp_hi", ground_clamp_hi_);
        declare_parameter<double>("ground_init", ground_init_);
        declare_parameter<int>("n_min_near", n_min_near_);
        declare_parameter<int>("n_min_mid", n_min_mid_);
        declare_parameter<int>("n_min_far", n_min_far_);
        declare_parameter<double>("near_dist", near_dist_);
        declare_parameter<double>("mid_dist", mid_dist_);
        declare_parameter<double>("delta_hit", delta_hit_);
        declare_parameter<double>("log_odds_cap", log_odds_cap_);
        declare_parameter<double>("log_odds_floor", log_odds_floor_);
        declare_parameter<double>("decay_tau", decay_tau_);
        declare_parameter<double>("occ_thresh", occ_thresh_);
        declare_parameter<double>("rate_hz", rate_hz_);
        declare_parameter<std::string>("cloud_topic", cloud_topic_);
        declare_parameter<std::string>("odom_topic", odom_topic_);
        declare_parameter<std::string>("costmap_topic", costmap_topic_);
        declare_parameter<std::string>("frame_id", frame_id_);
        declare_parameter<std::string>("world_file", world_file_);
        declare_parameter<std::string>("odom_frame", odom_frame_);

        resolution_ = get_parameter("resolution").as_double();
        width_m_ = get_parameter("width_m").as_double();
        height_m_ = get_parameter("height_m").as_double();
        x_offset_ = get_parameter("x_offset").as_double();
        y_offset_ = get_parameter("y_offset").as_double();
        h_climb_ = get_parameter("h_climb").as_double();
        ground_clamp_lo_ = get_parameter("ground_clamp_lo").as_double();
        ground_clamp_hi_ = get_parameter("ground_clamp_hi").as_double();
        ground_init_ = get_parameter("ground_init").as_double();
        n_min_near_ = get_parameter("n_min_near").as_int();
        n_min_mid_ = get_parameter("n_min_mid").as_int();
        n_min_far_ = get_parameter("n_min_far").as_int();
        near_dist_ = get_parameter("near_dist").as_double();
        mid_dist_ = get_parameter("mid_dist").as_double();
        delta_hit_ = get_parameter("delta_hit").as_double();
        log_odds_cap_ = get_parameter("log_odds_cap").as_double();
        log_odds_floor_ = get_parameter("log_odds_floor").as_double();
        decay_tau_ = get_parameter("decay_tau").as_double();
        occ_thresh_ = get_parameter("occ_thresh").as_double();
        rate_hz_ = get_parameter("rate_hz").as_double();
        cloud_topic_ = get_parameter("cloud_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        costmap_topic_ = get_parameter("costmap_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        world_file_ = get_parameter("world_file").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
    }

    struct BoxObstacle {
        double cx, cy, cz;  // center pose
        double sx, sy, sz;  // size
    };

    void loadStaticPrior() {
        RCLCPP_INFO(get_logger(), "Loading static prior from: %s", world_file_.c_str());

        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(world_file_.c_str()) != tinyxml2::XML_SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Failed to load world file: %s", world_file_.c_str());
            return;
        }

        std::vector<BoxObstacle> boxes;

        // Parse all <model> elements
        auto* world_elem = doc.FirstChildElement("sdf");
        if (!world_elem) { RCLCPP_ERROR(get_logger(), "No <sdf> root"); return; }
        auto* world = world_elem->FirstChildElement("world");
        if (!world) { RCLCPP_ERROR(get_logger(), "No <world> element"); return; }

        for (auto* model = world->FirstChildElement("model");
            model; model = model->NextSiblingElement("model"))
        {
            const char* name = model->Attribute("name");
            if (!name) continue;

            // Skip non-obstacle models (spawn zones, control zone, lights)
            std::string nstr(name);
            if (nstr.find("spawn") != std::string::npos ||
                nstr.find("control") != std::string::npos ||
                nstr.find("light") != std::string::npos ||
                nstr.find("floor") != std::string::npos)
                continue;

            auto* pose_elem = model->FirstChildElement("pose");
            auto* link = model->FirstChildElement("link");
            if (!pose_elem || !link) continue;

            std::string pose_str = pose_elem->GetText() ? pose_elem->GetText() : "";
            double px = 0, py = 0, pz = 0, roll = 0, pitch = 0, yaw = 0;
            std::istringstream iss(pose_str);
            iss >> px >> py >> pz >> roll >> pitch >> yaw;

            // Find first box geometry
            for (auto* collision = link->FirstChildElement("collision");
                collision; collision = collision->NextSiblingElement("collision"))
            {
                auto* geometry = collision->FirstChildElement("geometry");
                if (!geometry) continue;
                auto* box = geometry->FirstChildElement("box");
                if (!box) continue;
                auto* size_elem = box->FirstChildElement("size");
                if (!size_elem) continue;

                std::string size_str = size_elem->GetText() ? size_elem->GetText() : "";
                double sx = 0, sy = 0, sz = 0;
                std::istringstream siss(size_str);
                siss >> sx >> sy >> sz;

                boxes.push_back({px, py, pz, sx, sy, sz});
                break;
            }
        }

        RCLCPP_INFO(get_logger(), "Parsed %zu static obstacles from world file", boxes.size());

        // Static prior is directly in map (world) frame — no transform needed
        int prior_count = 0;
        for (const auto& b : boxes) {
            // Box half-extents
            double hx = b.sx / 2.0;
            double hy = b.sy / 2.0;

            // Rasterize box into grid (coordinates already in map frame)
            double x0 = b.cx - hx - origin_x_;
            double y0 = b.cy - hy - origin_y_;
            double x1 = b.cx + hx - origin_x_;
            double y1 = b.cy + hy - origin_y_;

            int i0 = std::max(0, static_cast<int>(std::floor(x0 / resolution_)));
            int j0 = std::max(0, static_cast<int>(std::floor(y0 / resolution_)));
            int i1 = std::min(grid_w_ - 1, static_cast<int>(std::floor(x1 / resolution_)));
            int j1 = std::min(grid_h_ - 1, static_cast<int>(std::floor(y1 / resolution_)));

            for (int j = j0; j <= j1; ++j) {
                for (int i = i0; i <= i1; ++i) {
                    static_prior_[j * grid_w_ + i] = true;
                    prior_count++;
                }
            }
        }

        RCLCPP_INFO(get_logger(), "Static prior: %d cells marked occupied", prior_count);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        try {
            auto tf = tf_buffer_->lookupTransform(
                frame_id_, odom_frame_, tf2::TimePointZero);
            double px = msg->pose.pose.position.x;
            double py = msg->pose.pose.position.y;
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;
            double yaw = 2.0 * std::atan2(q.z, q.w);
            double cos_y = std::cos(yaw), sin_y = std::sin(yaw);
            robot_x_ = cos_y * px - sin_y * py + t.x;
            robot_y_ = sin_y * px + cos_y * py + t.y;
            have_odom_ = true;
            tf_ready_ = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "odomCallback: TF lookup failed: %s", ex.what());
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Look up transform from odom_frame to frame_id (map)
        Eigen::Affine3d tf_odom_to_map;
        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                frame_id_, odom_frame_, tf2::TimePointZero);
            const auto& t = tf_msg.transform.translation;
            const auto& q = tf_msg.transform.rotation;
            double yaw = 2.0 * std::atan2(q.z, q.w);
            tf_odom_to_map = Eigen::Affine3d(
                Eigen::Translation3d(t.x, t.y, t.z) *
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            tf_ready_ = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "cloudCallback: TF lookup failed: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(*msg, pc);

        std::fill(frame_min_z_.begin(), frame_min_z_.end(), 1e9f);
        std::fill(hit_counts_.begin(), hit_counts_.end(), 0);

        // Collect Z values for percentile-based ground estimation
        std::vector<float> frame_zs;
        frame_zs.reserve(pc.points.size());

        for (const auto& p : pc.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            // Transform point from odom_frame to map frame
            Eigen::Vector3d pt(p.x, p.y, p.z);
            pt = tf_odom_to_map * pt;

            const double gx = pt.x() - origin_x_;
            const double gy = pt.y() - origin_y_;
            if (gx < 0 || gy < 0) continue;

            const int i = static_cast<int>(gx / resolution_);
            const int j = static_cast<int>(gy / resolution_);
            if (i < 0 || i >= grid_w_ || j < 0 || j >= grid_h_) continue;

            const int idx = j * grid_w_ + i;
            if (pt.z() < frame_min_z_[idx])
                frame_min_z_[idx] = pt.z();
            frame_zs.push_back(pt.z());

            const float h = pt.z() - global_ground_z_;
            if (h > h_climb_)
                hit_counts_[idx]++;
        }

        // Update global ground estimate using 10th percentile Z + EMA
        if (!frame_zs.empty()) {
            std::nth_element(frame_zs.begin(),
                             frame_zs.begin() + frame_zs.size() / 10,
                             frame_zs.end());
            const float p10 = frame_zs[frame_zs.size() / 10];
            const float alpha = 0.1f;
            float gz = alpha * p10 + (1.0f - alpha) * global_ground_z_;
            gz = std::clamp(gz, (float)ground_clamp_lo_, (float)ground_clamp_hi_);
            global_ground_z_ = gz;
        }

        // Log-odds update with distance-normalized N_min
        for (int j = 0; j < grid_h_; ++j) {
            for (int i = 0; i < grid_w_; ++i) {
                const int idx = j * grid_w_ + i;
                if (hit_counts_[idx] == 0) continue;

                const double cx = origin_x_ + (i + 0.5) * resolution_;
                const double cy = origin_y_ + (j + 0.5) * resolution_;
                const double dist = std::hypot(cx - robot_x_, cy - robot_y_);

                int n_min;
                if (dist < near_dist_) n_min = n_min_near_;
                else if (dist < mid_dist_) n_min = n_min_mid_;
                else n_min = n_min_far_;

                if (hit_counts_[idx] >= n_min) {
                    log_odds_[idx] += delta_hit_;
                    if (log_odds_[idx] > log_odds_cap_)
                        log_odds_[idx] = log_odds_cap_;
                }
            }
        }
    }

    void publishCostmap() {
        if (!have_odom_) return;

        // Time-based decay (use steady clock for consistent dt regardless of sim time)
        const auto now_steady = std::chrono::steady_clock::now();
        if (last_decay_wall_.time_since_epoch().count() == 0)
            last_decay_wall_ = now_steady;
        const double dt = std::chrono::duration<double>(now_steady - last_decay_wall_).count();
        last_decay_wall_ = now_steady;

        if (dt > 0 && decay_tau_ > 0) {
            const float decay = static_cast<float>(dt / decay_tau_);
            for (auto& lo : log_odds_) {
                lo -= decay;
                if (lo < log_odds_floor_) lo = log_odds_floor_;
            }
        }

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = frame_id_;
        grid.header.stamp = get_clock()->now();
        grid.info.resolution = static_cast<float>(resolution_);
        grid.info.width = grid_w_;
        grid.info.height = grid_h_;
        grid.info.origin.position.x = origin_x_;
        grid.info.origin.position.y = origin_y_;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(grid_w_ * grid_h_);

        for (int idx = 0; idx < grid_w_ * grid_h_; ++idx) {
            grid.data[idx] = (static_prior_[idx] || log_odds_[idx] > occ_thresh_) ? 100 : 0;
        }

        // Debug: log stats every 50 ticks (~5s at 10Hz)
        static int tick = 0;
        if (++tick % 50 == 0) {
            int occ = 0;
            float max_lo = -1e9f, min_lo = 1e9f;
            for (int idx = 0; idx < grid_w_ * grid_h_; ++idx) {
                if (grid.data[idx] == 100) occ++;
                if (log_odds_[idx] > max_lo) max_lo = log_odds_[idx];
                if (log_odds_[idx] < min_lo) min_lo = log_odds_[idx];
            }
            RCLCPP_INFO(get_logger(),
                "DBG ground_z=%.3f occ=%d/%d max_lo=%.2f min_lo=%.2f dt=%.3f decay=%.3f",
                global_ground_z_, occ, grid_w_*grid_h_, max_lo, min_lo, dt, dt/decay_tau_);
        }

        costmap_pub_->publish(grid);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityMapper>());
    rclcpp::shutdown();
    return 0;
}
