#include "omron_b5l_tof_front.hpp"

int ShutDownFlag;

INT32 Tof_output_format = 0;

std::string Topic = "test";

/**
 * @brief Construct a new omron b5l tof::omron b5l tof object
 */
omron_b5l_tof::omron_b5l_tof() : Node("omron_b5l_tof")
{
    ConfigParam conf;
    this->load_paramiters(conf);
    RCLCPP_INFO(this->get_logger(), "InitDevice: %s", conf.device.c_str()); // device_ is a member variable of omron_b5l_tof
    int Ret = b5l_tof_device::Init(conf, 921600, Tof_output_format); // 921600 default baudrate
    if (Ret == 0)
    {
		if (Tof_output_format == 257 || Tof_output_format == 258)
		{
			Topic = PUB_TOPIC_FRMT_257_258;
		}
		else
		{
			Topic = PUB_TOPIC_FRMT_001_002;
		}
    }

    frame_id = conf.frame_id;
    ShutDownFlag = 0;

    // publish pointcloud2
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(Topic, 10);
    timer_ = this->create_wall_timer(33ms, std::bind(&omron_b5l_tof::timer_callback, this));
    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
}

void omron_b5l_tof::timer_callback(){
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    int Ret = b5l_tof_device::Run(&cloud_);
    if (Ret == 0)
    {
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = frame_id;
        pc2_msg_->header.stamp = this->now();
        publisher_->publish(*pc2_msg_);
    }
    else
    {
        ShutDownFlag = 1;
        RCLCPP_ERROR(this->get_logger(), "Run error");
        terminate_application();
    }
}

/**
 * @brief Load paramiters from yaml file
 *  and print them to the console
 */
void omron_b5l_tof::load_paramiters(ConfigParam &conf)
{
    /**
     * @brief Load paramiters from yaml file
     */
    RCLCPP_INFO(this->get_logger(), "omron_b5l_tof node started");
    this->declare_parameter("frame_id", "omron_b5l_tof");
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("frequency", 20.0);
    this->declare_parameter("IMAGE_PATH", "./");
    this->declare_parameter("FOV_LIMITATION", 1);
    this->declare_parameter("OUTPUT_FORMAT", 257);
    this->declare_parameter("OPERATION_MODE", 0);
    this->declare_parameter("EXPOSURE_FPS", std::vector<int64_t>({850, 0}));
    this->declare_parameter("T3D_XYZ", std::vector<int64_t>({0, 0, 0}));
    this->declare_parameter("LED_FREQUENCY_ID", 0);
    this->declare_parameter("MIN_AMP_ALL", 0);
    this->declare_parameter("MIN_AMP_NEAR", 0);
    this->declare_parameter("OPERATION_CHECK_LED", 0);
    this->declare_parameter("RESPONSE_SPEED", std::vector<int64_t>({16, 0}));
    this->declare_parameter("ENR_TRESHOLD", 350);

    /**
     * @brief Get paramiters from yaml file
     */
    conf.frame_id = this->get_parameter("frame_id").as_string();
    conf.device = this->get_parameter("device").as_string();
    conf.frequency = this->get_parameter("frequency").as_double();
    conf.image_path = this->get_parameter("IMAGE_PATH").as_string();
    conf.fov_limit = this->get_parameter("FOV_LIMITATION").as_int();
    conf.output_format = this->get_parameter("OUTPUT_FORMAT").as_int();
    conf.operation_mode = this->get_parameter("OPERATION_MODE").as_int();
    conf.exposure_fps = this->get_parameter("EXPOSURE_FPS").as_integer_array();
    conf.t3d_xyz = this->get_parameter("T3D_XYZ").as_integer_array();
    conf.led_frequency_id = this->get_parameter("LED_FREQUENCY_ID").as_int();
    conf.min_amp_all = this->get_parameter("MIN_AMP_ALL").as_int();
    conf.min_amp_near = this->get_parameter("MIN_AMP_NEAR").as_int();
    conf.operation_check_led = this->get_parameter("OPERATION_CHECK_LED").as_int();
    conf.response_speed = this->get_parameter("RESPONSE_SPEED").as_integer_array();
    conf.enr_treshold = this->get_parameter("ENR_TRESHOLD").as_int();

    /**
     * @brief Print paramiters to the console
     */
    RCLCPP_INFO(this->get_logger(), "======== [PARAMETERS] ========");
    RCLCPP_INFO(this->get_logger(), "[frame_id]: %s", conf.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "[device]: %s", conf.device.c_str());
    RCLCPP_INFO(this->get_logger(), "[frequency]: %f", conf.frequency);
    RCLCPP_INFO(this->get_logger(), "[IMAGE_PATH]: %s", conf.image_path.c_str());
    RCLCPP_INFO(this->get_logger(), "[FOV_LIMITATION]: %d", conf.fov_limit);
    RCLCPP_INFO(this->get_logger(), "[OUTPUT_FORMAT]: %d", conf.output_format);
    RCLCPP_INFO(this->get_logger(), "[OPERATION_MODE]: %d", conf.operation_mode);
    RCLCPP_INFO(this->get_logger(), "[EXPOSURE_FPS]: %ld, %ld", conf.exposure_fps[0], conf.exposure_fps[1]);
    RCLCPP_INFO(this->get_logger(), "[T3D_XYZ]: %ld, %ld, %ld", conf.t3d_xyz[0], conf.t3d_xyz[1], conf.t3d_xyz[2]);
    RCLCPP_INFO(this->get_logger(), "[LED_FREQUENCY_ID]: %d", conf.led_frequency_id);
    RCLCPP_INFO(this->get_logger(), "[MIN_AMP_ALL]: %d", conf.min_amp_all);
    RCLCPP_INFO(this->get_logger(), "[MIN_AMP_NEAR]: %d", conf.min_amp_near);
    RCLCPP_INFO(this->get_logger(), "[OPERATION_CHECK_LED]: %d", conf.operation_check_led);
    RCLCPP_INFO(this->get_logger(), "[RESPONSE_SPEED]: %ld, %ld", conf.response_speed[0], conf.response_speed[1]);
    RCLCPP_INFO(this->get_logger(), "[ENR_TRESHOLD]: %d", conf.enr_treshold);
    RCLCPP_INFO(this->get_logger(), "=============================");
}

/**
 * @brief This function is for stop communication and shutdown application
 * 
*/
void omron_b5l_tof::terminate_application()
{
    b5l_tof_device::Stop();
    rclcpp::shutdown();
}

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ShutDownFlag = 0;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omron_b5l_tof>();
    rclcpp::spin(node);
    if (ShutDownFlag == 0){
        omron_b5l_tof::terminate_application();
    }
    return 0;
}