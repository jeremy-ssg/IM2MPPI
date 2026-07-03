#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace {

class LocalStaticMapNode {
public:
    LocalStaticMapNode() : pnh_("~") {
        pnh_.param<std::string>("global_static_map_topic", globalStaticTopic_,
                                "/dynamic_map/inflated_voxel_map");
        pnh_.param<std::string>("odom_topic", odomTopic_, "/CERLAB/quadcopter/odom");
        pnh_.param<std::string>("local_static_map_topic", localStaticTopic_,
                                "/tmpc/local_static_map");
        pnh_.param("perception_range", perceptionRange_, 5.0);
        pnh_.param("z_min", zMin_, -0.1);
        pnh_.param("z_max", zMax_, 2.5);
        pnh_.param("publish_rate", publishRate_, 10.0);
        pnh_.param("max_points", maxPoints_, 30000);
        pnh_.param("use_horizontal_range", useHorizontalRange_, true);
        if (zMax_ < zMin_) std::swap(zMax_, zMin_);
        perceptionRange_ = std::max(0.1, perceptionRange_);
        publishRate_ = std::max(1.0, publishRate_);
        maxPoints_ = std::max(100, maxPoints_);

        globalSub_ = nh_.subscribe(globalStaticTopic_, 1, &LocalStaticMapNode::cloudCB, this);
        odomSub_ = nh_.subscribe(odomTopic_, 20, &LocalStaticMapNode::odomCB, this);
        localPub_ = nh_.advertise<sensor_msgs::PointCloud2>(localStaticTopic_, 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publishRate_),
                                 &LocalStaticMapNode::timerCB, this);

        ROS_INFO("[local_static_map] global=%s odom=%s local=%s range=%.2fm z=[%.2f, %.2f]",
                 globalStaticTopic_.c_str(), odomTopic_.c_str(), localStaticTopic_.c_str(),
                 perceptionRange_, zMin_, zMax_);
    }

private:
    void cloudCB(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::vector<Eigen::Vector3d> pts;
        pts.reserve(cloud.points.size());
        for (const auto& p : cloud.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            pts.emplace_back(p.x, p.y, p.z);
        }

        std::lock_guard<std::mutex> lk(mutex_);
        globalPoints_.swap(pts);
        frameId_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        haveCloud_ = true;
    }

    void odomCB(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        currPos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
        haveOdom_ = true;
    }

    bool inRange(const Eigen::Vector3d& p, const Eigen::Vector3d& c) const {
        if (p.z() < zMin_ || p.z() > zMax_) return false;
        if (useHorizontalRange_) {
            return (p.head<2>() - c.head<2>()).squaredNorm()
                   <= perceptionRange_ * perceptionRange_;
        }
        return (p - c).squaredNorm() <= perceptionRange_ * perceptionRange_;
    }

    void timerCB(const ros::TimerEvent&) {
        std::vector<Eigen::Vector3d> global;
        Eigen::Vector3d curr;
        std::string frame;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!haveCloud_ || !haveOdom_) return;
            global = globalPoints_;
            curr = currPos_;
            frame = frameId_;
        }

        int eligible = 0;
        for (const auto& p : global) {
            if (inRange(p, curr)) ++eligible;
        }
        const int stride = std::max(1, (int)std::ceil((double)eligible / (double)maxPoints_));

        pcl::PointCloud<pcl::PointXYZ> local;
        local.header.frame_id = frame;
        local.points.reserve((size_t)std::min(eligible, maxPoints_));
        int kept = 0;
        int seen = 0;
        for (const auto& p : global) {
            if (!inRange(p, curr)) continue;
            if ((seen++ % stride) != 0) continue;
            pcl::PointXYZ pt;
            pt.x = (float)p.x();
            pt.y = (float)p.y();
            pt.z = (float)p.z();
            local.points.push_back(pt);
            if (++kept >= maxPoints_) break;
        }
        local.width = (uint32_t)local.points.size();
        local.height = 1;
        local.is_dense = true;

        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(local, out);
        out.header.frame_id = frame;
        out.header.stamp = ros::Time::now();
        localPub_.publish(out);

        ROS_INFO_THROTTLE(2.0,
            "[local_static_map] published %zu/%d points around (%.2f, %.2f, %.2f)",
            local.points.size(), eligible, curr.x(), curr.y(), curr.z());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber globalSub_;
    ros::Subscriber odomSub_;
    ros::Publisher localPub_;
    ros::Timer timer_;

    std::mutex mutex_;
    std::vector<Eigen::Vector3d> globalPoints_;
    Eigen::Vector3d currPos_ = Eigen::Vector3d::Zero();
    std::string frameId_ = "map";
    bool haveCloud_ = false;
    bool haveOdom_ = false;

    std::string globalStaticTopic_;
    std::string odomTopic_;
    std::string localStaticTopic_;
    double perceptionRange_ = 5.0;
    double zMin_ = -0.1;
    double zMax_ = 2.5;
    double publishRate_ = 10.0;
    int maxPoints_ = 30000;
    bool useHorizontalRange_ = true;
};

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_static_map_node");
    LocalStaticMapNode node;
    ros::spin();
    return 0;
}
