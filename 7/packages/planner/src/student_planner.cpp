#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "planner_interfaces/srv/plan_path.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace planner {
    class StudentPlanner : public nav2_core::GlobalPlanner {
      public:
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
            std::string,
            std::shared_ptr<tf2_ros::Buffer>,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override {
            auto node = parent.lock();
            if (!node) {
                throw std::runtime_error("Planner server expired");
            }

            clock_ = node->get_clock();
            costmap_ros_ = std::move(costmap_ros);
            // use a separate node so the bridge can spin its own service client
            service_node_ = rclcpp::Node::make_shared("student_planner_bridge");
            client_ = service_node_->create_client<planner_interfaces::srv::PlanPath>(
                "/student_plan"
            );
        }

        void cleanup() override {
            client_.reset();
            service_node_.reset();
            costmap_ros_.reset();
        }

        void activate() override {}

        void deactivate() override {}

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped& start,
            const geometry_msgs::msg::PoseStamped& goal
        ) override {
            using namespace std::chrono_literals;
            auto request = std::make_shared<planner_interfaces::srv::PlanPath::Request>();
            request->start = start;
            request->goal = goal;
            auto& message = request->costmap;
            message.header.frame_id = costmap_ros_->getGlobalFrameID();
            message.header.stamp = clock_->now();
            auto* map = costmap_ros_->getCostmap();
            // copy metadata and cells under one lock to avoid mixed map revisions
            {
                std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
                message.metadata.size_x = map->getSizeInCellsX();
                message.metadata.size_y = map->getSizeInCellsY();
                if (!message.metadata.size_x || !message.metadata.size_y) {
                    throw std::runtime_error("Global costmap is empty");
                }
                message.metadata.resolution = map->getResolution();
                message.metadata.origin.position.x = map->getOriginX();
                message.metadata.origin.position.y = map->getOriginY();
                message.metadata.origin.orientation.w = 1.0;
                const auto* data = map->getCharMap();
                message.data.assign(
                    data, data + message.metadata.size_x * message.metadata.size_y
                );
            }

            if (!client_->wait_for_service(2s)) {
                throw std::runtime_error("A* service is unavailable");
            }

            auto future = client_->async_send_request(request);
            // spin the client node to receive the python service reply
            if (rclcpp::spin_until_future_complete(service_node_, future, 5s)
                != rclcpp::FutureReturnCode::SUCCESS) {
                throw std::runtime_error("A* service timed out");
            }

            auto response = future.get();
            // reject invalid replies before returning a path to nav2
            if (!response->success || response->path.poses.empty()
                || response->path.header.frame_id != message.header.frame_id) {
                throw std::runtime_error(
                    response->error.empty() ? "A* returned no valid path" : response->error
                );
            }

            return response->path;
        }

      private:
        rclcpp::Clock::SharedPtr clock_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        rclcpp::Node::SharedPtr service_node_;
        rclcpp::Client<planner_interfaces::srv::PlanPath>::SharedPtr client_;
    };
// end namespace planner
}

PLUGINLIB_EXPORT_CLASS(planner::StudentPlanner, nav2_core::GlobalPlanner)
