#include <holonomic_controller/holonomic_controller_node.h>

std::string JsonAsString(const Json::Value &json)
{
    std::string result;
    Json::StreamWriterBuilder wbuilder;

    wbuilder["indentation"] = ""; // Optional
    result = Json::writeString(wbuilder, json);
    return result;
}

std::string map_to_string_as_json(std::map<std::string, float> &m)
{
    Json::Value yaml_json;
    std::string output_data;

    for (auto it = m.cbegin(); it != m.cend(); it++)
    {
        yaml_json[it->first] = it->second;
    }
    output_data = JsonAsString(yaml_json);
    return output_data;
}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel)
{
    ROS_INFO("GOT CMD VEL");
    w1 = (cmd_vel.linear.x - cmd_vel.linear.y - ((lx + ly) * cmd_vel.angular.z)) / wheel_radius;
    w2 = (cmd_vel.linear.x + cmd_vel.linear.y + ((lx + ly) * cmd_vel.angular.z)) / wheel_radius;
    w3 = (cmd_vel.linear.x + cmd_vel.linear.y - ((lx + ly) * cmd_vel.angular.z)) / wheel_radius;
    w4 = (cmd_vel.linear.x - cmd_vel.linear.y + ((lx + ly) * cmd_vel.angular.z)) / wheel_radius;
    std::map<std::string, float> mapped_speed = {
        {"front_left", w1},
        {"front_right", w2},
        {"back_left", w3},
        {"back_right", w4}};
    msg.data = map_to_string_as_json(mapped_speed);
    ROS_INFO_STREAM("distribuited speed " << msg.data);
    speed_pub.publish(msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "holonomic_controller_node");
    ros::NodeHandle n;

    //comment the blow line to print the logs on terminal
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    speed_pub = n.advertise<std_msgs::String>("/distributed_speed", 1);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);

    ros::spin();

    return 0;
}
