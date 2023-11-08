#include <holonomic_controller/holonomic_odometry.h>

void reset_got_data()
{
    new_fl = 0;
    new_fr = 0;
    new_bl = 0;
    new_br = 0;
}

void update_last_enc_data()
{
    last_back_left_enc = back_left_enc;
    last_back_right_enc = back_right_enc;
    last_front_left_enc = front_left_enc;
    last_front_right_enc = front_right_enc;
}

void publish_transform(float x, float y, float z)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

void relative_orientation()
{
    th -= init_th;
}

void update_frult()
{
    if (front_right_enc < encoder_low_wrap && last_front_right_enc > encoder_high_wrap)
    {
        frult += 1;
    }
    if (front_right_enc > encoder_high_wrap && last_front_right_enc < encoder_low_wrap)
    {
        frult -= 1;
    }
    front_right_enc += frult*(encoder_max-encoder_min);
}

void update_flult()
{
    if (front_left_enc < encoder_low_wrap && last_front_left_enc > encoder_high_wrap)
    {
        flult += 1;
    }
    if (front_left_enc > encoder_high_wrap && last_front_left_enc < encoder_low_wrap)
    {
        flult -= 1;
    }
    front_left_enc += flult*(encoder_max-encoder_min);
}

void update_brult()
{
    if (back_right_enc < encoder_low_wrap && last_back_right_enc > encoder_high_wrap)
    {
        brult += 1;
    }
    if (back_right_enc > encoder_high_wrap && last_back_right_enc < encoder_low_wrap)
    {
        brult -= 1;
    }
    back_right_enc += brult*(encoder_max-encoder_min);
}

void update_blult()
{
    if (back_left_enc < encoder_low_wrap && last_back_left_enc > encoder_high_wrap)
    {
        blult += 1;
    }
    if (back_left_enc > encoder_high_wrap && last_back_left_enc < encoder_low_wrap)
    {
        blult -= 1;
    }
    back_left_enc += blult*(encoder_max-encoder_min);
}

void front_left_enc_cb(const std_msgs::Int64 &encoder)
{
    front_left_enc = encoder.data;
    new_fl = 1;
}

void front_right_enc_cb(const std_msgs::Int64 &encoder)
{
    front_right_enc = encoder.data; //-1 * encoder.data;
    new_fr = 1;
}

void back_left_enc_cb(const std_msgs::Int64 &encoder)
{
    back_left_enc = encoder.data;
    new_bl = 1;
}

void back_right_enc_cb(const std_msgs::Int64 &encoder)
{
    back_right_enc = encoder.data; //-1 * encoder.data;
    new_br = 1;
}

void imu_cb(const std_msgs::Float32 &imu)
{
    th = -1 * imu.data;
    if (!got_imu)
    {
        init_th = th;
    }
    relative_orientation();
    got_imu = 1;
}

void publish_odom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, th);
    odom.pose.pose.orientation = tf2::toMsg(quat);

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);
}

float compute_distance(int new_ticks, int old_ticks)
{
    float delta = new_ticks - old_ticks;
    return delta /ticks_meter;
}

void timerCallback(const ros::TimerEvent &)
{   
    if (got_imu && new_bl && new_br && new_fr && new_fl)
    {
        update_blult();
        update_brult();
        update_flult();
        update_frult();
        if (init)
        {
            current_time = ros::Time::now();
            last_time = ros::Time::now();
            update_last_enc_data();
            ROS_INFO_STREAM("INITIAL ENCODERS " << front_left_enc << " " << front_right_enc << " " << back_left_enc << " " << back_right_enc);
            init = 0;
        }
        else
        {
            current_time = ros::Time::now();

            float dt = (current_time - last_time).toSec();

            w1 = compute_distance(last_front_left_enc,front_left_enc)/dt;
            w2 = compute_distance(last_front_right_enc,front_right_enc)/dt;
            w3 = compute_distance(last_back_left_enc,back_left_enc)/dt;
            w4 = compute_distance(last_back_right_enc,back_right_enc)/dt;
            ROS_INFO_STREAM("ENCODERS " << front_left_enc << " " << front_right_enc << " " << back_left_enc << " " << back_right_enc);
            ROS_INFO_STREAM("MOTOR SPEEDS " << w1 << " " << w2 << " " << w3 << " " << w4 << " IMU " << th);

            // inverted motors
            w2 = -1 * w2;
            w4 = -1 * w4;

            vx = (w1 + w2 + w3 + w4) * (wheel_radius / 4) ;
            vy = (-w1 + w2 + w3 - w4) * (wheel_radius / 4);
            vth = (-w1 + w2 - w3 + w4) * (wheel_radius / (4 * (lx + ly)));

            vy *= 0.9448818897637796; //check lateral_diff_calculation.py for this multiplication factor.

            ROS_INFO_STREAM("VELOCITY " << vx << " " << vy << " time " << dt);

            float delta_x = (vx * cos(th) * dt) - (vy * sin(th) * dt);
            float delta_y = (vx * sin(th) * dt) + (vy * cos(th) * dt);
            float delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;

            ROS_INFO_STREAM("DISTANCE " << x << " " << y);
            publish_transform(x, y, th);
            publish_odom();

            update_last_enc_data();
            last_time = current_time;
            reset_got_data();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "holonomic_odometry");
    ros::NodeHandle nh;

    //comment the blow line to print the logs on terminal
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber front_left_sub = nh.subscribe("/motor_front_left/encoder", 1, front_left_enc_cb);
    ros::Subscriber front_right_sub = nh.subscribe("/motor_front_right/encoder", 1, front_right_enc_cb);
    ros::Subscriber back_left_sub = nh.subscribe("/motor_back_left/encoder", 1, back_left_enc_cb);
    ros::Subscriber back_right_sub = nh.subscribe("/motor_back_right/encoder", 1, back_right_enc_cb);
    ros::Subscriber imu_sub = nh.subscribe("/orientation", 1, imu_cb);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback); // Manish: Duration changed to 0.034 from 0.1
    ros::spin();
}