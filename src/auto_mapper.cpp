//
// Created by omar on 2/5/24.
//
//
// Created by omar on 1/30/24.
//
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <filesystem>
#include <slam_toolbox/srv/detail/save_map__struct.hpp>
#include <fstream>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using nav_msgs::msg::OccupancyGrid;
using nav2_msgs::action::NavigateToPose;
using map_msgs::msg::OccupancyGridUpdate;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using namespace nav2_map_server;
using namespace slam_toolbox;
using std::chrono::steady_clock;

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoMapper : public Node {
public:
    AutoMapper()
            : Node("auto_mapper") {
        RCLCPP_INFO(get_logger(), "AutoMapper started...");

        poseSubscription_ = create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, bind(&AutoMapper::poseTopicCallback, this, _1));

        mapSubscription_ = create_subscription<OccupancyGrid>(
                "/map", 10, bind(&AutoMapper::updateFullMap, this, _1));

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10);
        poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose");

        poseNavigator_->wait_for_action_server();
        RCLCPP_INFO(get_logger(), "AutoMapper poseNavigator_");
        declare_parameter("map_path", rclcpp::PARAMETER_STRING);
        get_parameter("map_path", mapPath_);
    }

private:
    double minFrontierSize_ = 0.8;
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
    Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    MarkerArray markersMsg_;
    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    bool isExploring_ = false;
    int markerId_;
    unordered_set<string> blackList_;
    string mapPath_;


    Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_;
    PoseWithCovarianceStamped::UniquePtr pose_;


    struct Frontier {
        Point centroid;
        vector<Point> points;
    };

    void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        pose_ = move(pose);
        RCLCPP_INFO(get_logger(), "poseTopicCallback...");
    }

    array<unsigned char, 256> initTranslationTable() {
        array<unsigned char, 256> cost_translation_table{};

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = FREE_SPACE;
        cost_translation_table[99] = 253;
        cost_translation_table[100] = LETHAL_OBSTACLE;
        cost_translation_table[static_cast<unsigned char>(-1)] = NO_INFORMATION;

        return cost_translation_table;
    }

    array<unsigned char, 256> costTranslationTable_ = initTranslationTable();

    void updateFullMap(OccupancyGrid::UniquePtr occupancyGrid) {
        RCLCPP_INFO(get_logger(), "updateFullMap...");
        if (pose_ == nullptr) { return; }
        const auto occupancyGridInfo = occupancyGrid->info;
        unsigned int size_in_cells_x = occupancyGridInfo.width;
        unsigned int size_in_cells_y = occupancyGridInfo.height;
        double resolution = occupancyGridInfo.resolution;
        double origin_x = occupancyGridInfo.origin.position.x;
        double origin_y = occupancyGridInfo.origin.position.y;

        RCLCPP_INFO(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = costTranslationTable_[cell_cost];
        }

        explore();
    }

    void drawMarkers(const vector<Frontier> &frontiers) {
        for (const auto &frontier: frontiers) {
            RCLCPP_INFO(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
            ColorRGBA green;
            green.r = 0;
            green.g = 1.0;
            green.b = 0;
            green.a = 1.0;

            vector<Marker> &markers = markersMsg_.markers;
            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::ADD;
            m.ns = "frontiers";
            m.id = ++markerId_;
            m.type = Marker::SPHERE;
            m.pose.position = frontier.centroid;
            m.scale.x = 0.5;
            m.scale.y = 0.5;
            m.scale.z = 0.5;
            m.color = green;
            markers.push_back(m);
            markerArrayPublisher_->publish(markersMsg_);
        }
    }

    void clearMarkers() {
        for (auto &m: markersMsg_.markers) {
            m.action = Marker::DELETE;
        }
        markerArrayPublisher_->publish(markersMsg_);
    }

    void stop() {
        RCLCPP_INFO(get_logger(), "Stopped...");
        poseNavigator_->async_cancel_all_goals();
        saveMap();
    }

    void explore() {
        if (isExploring_) { return; }
        auto frontiers = findFrontiers();
        if (frontiers.empty()) {
            RCLCPP_WARN(get_logger(), "NO BOUNDARIES FOUND!!");
            stop();
            return;
        }
        const auto frontier = frontiers[0];
        drawMarkers(frontiers);
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = frontier.centroid;
        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontier.centroid.x, frontier.centroid.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this, &frontier](
                const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
                isExploring_ = true;
            } else {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
                blackList_.insert(to_string(frontier.centroid.x) + "," + to_string(frontier.centroid.y));
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            isExploring_ = false;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached");
                    saveMap();
                    clearMarkers();
                    explore();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    break;
            }
        };
        poseNavigator_->async_send_goal(goal, send_goal_options);
    }

    void saveMap() {
        auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>(
                "/slam_toolbox/serialize_map");
        auto serializePoseGraphRequest =
                std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
        serializePoseGraphRequest->filename = mapPath_;
        auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest);

        auto map_saver = create_client<slam_toolbox::srv::SaveMap>(
                "/slam_toolbox/save_map");
        auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        saveMapRequest->name.data = mapPath_;
        auto saveMapResult = map_saver->async_send_request(saveMapRequest);
    }

    vector<unsigned int> nhood4(unsigned int idx) {
        // get 4-connected neighbourhood indexes, check for edge of map
        vector<unsigned int> out;

        unsigned int size_x_ = costmap_.getSizeInCellsX(),
                size_y_ = costmap_.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) {
            RCLCPP_WARN(get_logger(), "Evaluating nhood for offmap point");
            return out;
        }

        if (idx % size_x_ > 0) {
            out.push_back(idx - 1);
        }
        if (idx % size_x_ < size_x_ - 1) {
            out.push_back(idx + 1);
        }
        if (idx >= size_x_) {
            out.push_back(idx - size_x_);
        }
        if (idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + size_x_);
        }
        return out;
    }

    vector<unsigned int> nhood8(unsigned int idx) {
        // get 8-connected neighbourhood indexes, check for edge of map
        vector<unsigned int> out = nhood4(idx);

        unsigned int size_x_ = costmap_.getSizeInCellsX(),
                size_y_ = costmap_.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) {
            return out;
        }

        if (idx % size_x_ > 0 && idx >= size_x_) {
            out.push_back(idx - 1 - size_x_);
        }
        if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx - 1 + size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
            out.push_back(idx + 1 - size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + 1 + size_x_);
        }

        return out;
    }

    bool isNewFrontierCell(unsigned int idx,
                           const vector<bool> &frontier_flag) {
        auto map = costmap_.getCharMap();
        // check that cell is unknown and not already marked as frontier
        if (map[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }

        // frontier cells should have at least one cell in 4-connected neighbourhood
        // that is free
        for (unsigned int nbr: nhood4(idx)) {
            if (map[nbr] == FREE_SPACE) {
                return true;
            }
        }

        return false;
    }

    Frontier buildNewFrontier(unsigned int neighborCell,
                              unsigned int robotCell,
                              vector<bool> &frontier_flag) {
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;

        queue<unsigned int> bfs;
        bfs.push(neighborCell);

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: nhood8(idx)) {
                // check if neighbour is a potential frontier cell
                if (isNewFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.points.size();
        output.centroid.y /= output.points.size();
        return output;
    }

    vector<Frontier> findFrontiers() {
        vector<Frontier> frontier_list;
        const auto position = pose_->pose.pose.position;
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        auto map_ = costmap_.getCharMap();
        auto size_x_ = costmap_.getSizeInCellsX();
        auto size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);
        bfs.push(pos);
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // iterate over 4-connected neighbourhood
            for (unsigned nbr: nhood4(idx)) {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (map_[nbr] == FREE_SPACE && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour)
                } else if (isNewFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                    if (new_frontier.points.size() * costmap_.getResolution() >=
                        minFrontierSize_) {
                        frontier_list.push_back(new_frontier);
                    }
                }
            }
        }

        return frontier_list;
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<AutoMapper>());
    shutdown();
    return 0;
}