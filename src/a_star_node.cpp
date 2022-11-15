#include <iostream>
#include "AStar.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "boat_interfaces/msg/vector_array.hpp"

#include "a_star/buffer_subscriber.hpp"
#include "a_star/Timer.hpp"

#include <vector>

using namespace std::chrono_literals;

class A_star_node : public rclcpp::Node
{
public:
    explicit A_star_node(const rclcpp::NodeOptions & options)
        : Node("a_star", options), count_(0)
    {
        subscribe_from(this, end_points_ptr, "/end_points");
        subscribe_from(this, occupied_points_ptr, "/occupied_points");
        subscribe_from(this, map_size_ptr, "/map_size_ptr");

		this->path_section_publisher = this->create_publisher<boat_interfaces::msg::VectorArray>("path_section", 1);

        timer_ = this->create_wall_timer(50ms, std::bind(&A_star_node::timer_callback, this));
    }

private:
    void timer_callback()
    {
        if(!(this->map_size_ptr->has_msg()))
            return;
        // read the values

        this->end_points = *(this->end_points_ptr->take());
        this->occupied_points = *(this->occupied_points_ptr->take());
        this->map_size = *(this->map_size_ptr->take());
        // format them for class
        std::vector<AStar::Vec2i> occupied_points_vec2i;
        std::vector<AStar::Vec2i> end_points_vec2i;
        this->toVec2i(occupied_points_vec2i, this->occupied_points);
        this->toVec2i(end_points_vec2i, this->end_points);
        std::vector<AStar::Vec2i> path_vec2i;

        generate_path(path_vec2i, this->map_size, end_points_vec2i, occupied_points_vec2i);
        boat_interfaces::msg::VectorArray path_output;

        toVectorArray(path_output, path_vec2i);
        this->path_section_publisher->publish(path_output);

    }

    MsgSubscriber<boat_interfaces::msg::VectorArray>::UniquePtr end_points_ptr;
    MsgSubscriber<boat_interfaces::msg::VectorArray>::UniquePtr occupied_points_ptr;
    MsgSubscriber<geometry_msgs::msg::Vector3>::UniquePtr map_size_ptr;
    boat_interfaces::msg::VectorArray end_points;
    boat_interfaces::msg::VectorArray occupied_points;
    geometry_msgs::msg::Vector3 map_size;

    rclcpp::Publisher<boat_interfaces::msg::VectorArray>::SharedPtr path_section_publisher;

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    void toVec2i(std::vector<AStar::Vec2i> &output, boat_interfaces::msg::VectorArray &originalFormat)
    {
        for (int i = 0; i < int(originalFormat.vec3list.size()); ++i)
        {
            output.push_back({int(originalFormat.vec3list[i].x), int(originalFormat.vec3list[i].y)});
        }
    }
    void toVectorArray(boat_interfaces::msg::VectorArray &output, std::vector<AStar::Vec2i> &originalFormat)
    {
        for (int i = 0; i < int(originalFormat.size()); ++i)
        {
            geometry_msgs::msg::Vector3 theVec;
            theVec.x = originalFormat[i].x;
            theVec.y = originalFormat[i].y;
            theVec.z = 0;
            output.vec3list.push_back(theVec);
        }
    }
    void generate_path(std::vector<AStar::Vec2i> &path, geometry_msgs::msg::Vector3 &map_size, 
    std::vector<AStar::Vec2i> &end_points, std::vector<AStar::Vec2i> &occupied_points)
    {
        AStar::Generator generator;

        generator.setWorldSize({int(map_size.x), int(map_size.y)});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);

        for (int i = 0; i < int(occupied_points.size()); ++i)
        {
            generator.addCollision(occupied_points[i]);
        }
        
        auto temp_path = generator.findPath({end_points[0].x, end_points[0].y},
                                  {end_points[1].x, end_points[1].y});
        for (int i = 0; i < int(occupied_points.size()); ++i)
        {
            generator.removeCollision(occupied_points[i]);
        }
        for(int i = 0; i < int(temp_path.size()); i++){
            path.insert(path.begin(), temp_path[i]);
        }
    }
};

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
	rclcpp::NodeOptions options{};
	auto node = std::make_shared<A_star_node>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
    
}