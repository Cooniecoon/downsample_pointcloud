#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>



class PointcloudSubscriber : public rclcpp::Node
{
public:
    PointcloudSubscriber() : Node("pointcloud_downsample_node")
    {   
        this->declare_parameter("subscribe_topic_name", "/camera/depth/color/points");
        this->declare_parameter("publish_topic_name", "/camera/depth/color/points_downsampled");
        std::string sub_topic =this->get_parameter("subscribe_topic_name").get_parameter_value().get<std::string>();
        std::string pub_topic =this->get_parameter("publish_topic_name").get_parameter_value().get<std::string>();
        auto qos_profile = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sub_topic,qos_profile,std::bind(&PointcloudSubscriber::topic_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic,qos_profile);

        setSegmentationParam();
        initPtr();
    }
protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    sensor_msgs::msg::PointCloud2 filtered_msg;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_points_neg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_voxelized_;

    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inliers_;

    float voxelsize_ = 0.02;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::fromROSMsg(*msg, *cloud_);

        voxelize(cloud_, *pc_voxelized_, voxelsize_);
        // groundSegmentation();

        pcl::toROSMsg(*inlier_points_neg_, filtered_msg);
        publisher_->publish(filtered_msg);
    }

    void groundSegmentation()
    {
        seg_.setInputCloud(pc_voxelized_);
        seg_.segment(*inliers_, *coefficients_);

        extract_.setInputCloud(pc_voxelized_);
        extract_.setIndices(inliers_);
        extract_.filter(*inlier_points_neg_);
    }

    void setSegmentationParam()
    {
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setMaxIterations(1000);
        seg_.setDistanceThreshold(0.01);
        seg_.setRadiusLimits(0, 0.1);
        extract_.setNegative(true);
    }

    void initPtr()
    {
        cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        inlier_points_neg_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pc_voxelized_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        coefficients_ = std::make_shared<pcl::ModelCoefficients>();
        inliers_ = std::make_shared<pcl::PointIndices>();
    }

    void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src, pcl::PointCloud<pcl::PointXYZ>& pc_dst, double var_voxel_size)
    {
        voxel_filter_.setInputCloud(pc_src);
        voxel_filter_.setLeafSize(var_voxel_size, var_voxel_size,var_voxel_size);
        voxel_filter_.filter(pc_dst);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudSubscriber>());
    rclcpp::shutdown();
}
