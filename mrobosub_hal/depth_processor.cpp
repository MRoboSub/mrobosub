/**
 * Node to process the raw depth readings from the Arduino
 * Bar30 depth sensing code and republish them. 
 * 
 * Authors:
 * - Ivan Wei (ivanw8288@gmail.com)
 * - Ayan Chowdhury (ayanc@umich.edu)
 **/

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"

/** CONSTANTS **/
const std::string NODE_NAME = "/depth/depth_processor";
const std::string ARDUINO_DEPTH_TOPIC = "/depth/raw_depth";
const std::string OUTPUT_DEPTH_TOPIC = "/depth/depth";
const std::string RESET_DEPTH_SRV = "/depth/reset";

const uint32_t SUBSCRIBER_QUEUE_SIZE = 1000;
const uint32_t PUBLISHER_QUEUE_SIZE = 1000;

// Rate matches arduino publishing rate
const double PROCESS_RATE_HZ = 20;

/** Relative **/
// Used to track which depth is considered to be "zero".
bool initialized_rel = false;
float rel = 0;

/** Output **/
std_msgs::Float32 depth;

/**
 * Modifies output depth to be the raw depth processed through anything
 * needed; i.e., the relative "zero" depth is subtracted.
 * 
 * Is a callback for a subscriber to the arduino depth topic.
 * 
 * Parameters:
 * - raw_depth: the depth read from the arduino, passed in through callback 
 **/
void process_depth(const std_msgs::Float32& raw_depth)
{
    // If this is the first measurement after a reset, set the current depth as relative
    if (!initialized_rel)
    {   
        initialized_rel = true;
        rel = raw_depth.data;
    }

    // Set our depth output to the relative depth
    depth.data = raw_depth.data - rel;
}

/**
 * Resets the relative "zero" depth as well as the output depth.
 **/
void reset() 
{
    initialized_rel = false;
    rel = 0;
    depth.data = 0;
}

/**
 * Callback for reset service to wrap reset().
 * 
 * Parameters
 * - request: empty service request
 * - response: empty service response
 * Returns
 * - true
 **/
bool reset_service_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    reset();
    return true;
}

int main(int argc, char **argv)
{
    // Initialize node
    ros::NodeHandle nh;
    ros::init(argc, argv, NODE_NAME);
    
    // Setup rate
    ros::Rate rate(PROCESS_RATE_HZ);

    // Setup subscribers 
    ros::Subscriber raw_depth_sub = nh.subscribe(ARDUINO_DEPTH_TOPIC, SUBSCRIBER_QUEUE_SIZE, process_depth);
    ros::Publisher depth_pub = nh.advertise<std_msgs::Float32>(OUTPUT_DEPTH_TOPIC, PUBLISHER_QUEUE_SIZE);

    // Setup reset service
    ros::ServiceServer reset_srv_server = nh.advertiseService(RESET_DEPTH_SRV, reset_service_callback);

    reset();

    // Main publisher loop
    while (ros::ok())
    {   
        ros::spinOnce();

        if (initialized_rel)
        {
            depth_pub.publish(depth);
        }

        rate.sleep();
    }
}