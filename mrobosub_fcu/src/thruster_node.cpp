#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16MultiArray.h"

#include "ThrusterManager.hpp"
#include "Thruster.hpp"
#include "Screw.hpp"

#include <sstream>
#include <vector>
#include <map>
#include <cassert>
#include <iostream>

using namespace std;
using namespace XmlRpc;

class ThrusterNode {
public:
	ThrusterNode() {

	}

	void run() {
		ros::Subscriber surge_sub = n.subscribe("surge", 1, &ThrusterNode::surge_callback, this);
		ros::Subscriber sway_sub = n.subscribe("sway", 1, &ThrusterNode::sway_callback, this);
		ros::Subscriber heave_sub = n.subscribe("heave", 1, &ThrusterNode::heave_callback, this);
		ros::Subscriber yaw_sub = n.subscribe("yaw", 1, &ThrusterNode::yaw_callback, this);
		ros::Subscriber pitch_sub = n.subscribe("pitch", 1, &ThrusterNode::pitch_callback, this);
		ros::Subscriber roll_sub = n.subscribe("roll", 1, &ThrusterNode::roll_callback, this);

		thruster_manager = ThrusterManager{}; // TODO: From Parameter File

		XmlRpc::XmlRpcValue fit_params, thruster_params;
		n.getParam("/fits", fit_params);
		n.getParam("/thrusters", thruster_params);
		auto fits = parse_fits_param(fit_params);
		auto thrusters = parse_thrusters_param(thruster_params);

		// ros::Rate loop_rate(10);

		ros::spin();
	}

private:
	ros::NodeHandle n;
	ros::Publisher motor_pub = n.advertise<std_msgs::Int16MultiArray>("motor", 1);
	ThrusterManager thruster_manager;

	Screw<double> screw{0,0,0,0,0,0};

	static Thruster::QuadParams parse_quad_param(const XmlRpc::XmlRpcValue &value) {
		return Thruster::QuadParams(
			double(value["a"]),
			double(value["b"]),
			double(value["c"])
		);
	}

	static vector<Thruster> parse_thrusters_param(const XmlRpc::XmlRpcValue &value){
		vector<Thruster> thrusters;
		try{
			assert(value.getType() == XmlRpcValue::Type::TypeArray);
			auto &fit_param[]
			for(size_t i = 0; i < value.size(); ++i){
				auto
			}
		}
	}

	static vector<Thruster::PWMFit> parse_fits_param(const XmlRpc::XmlRpcValue &value) {
		vector<Thruster::PWMFit> fits;

		try {
			assert(value.getType() == XmlRpcValue::Type::TypeArray);

			for(size_t i = 0; i < value.size(); ++i) {
				auto &fit_param = value[i];
				auto v_param = fit_param["voltage"];
				// cout << v_param.getType() << endl;
				fits.push_back({
					double(v_param),
					parse_quad_param(fit_param["fwd"]),
					parse_quad_param(fit_param["rev"])
				});
			}
		} catch(const XmlRpcException &e) {
			cout << e.getMessage() << endl;
		}

		return fits;
	}

	void surge_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.surge() = msg.get()->data;
		publish_pwm();
	}

	void sway_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.sway() = msg.get()->data;
		publish_pwm();
	}

	void heave_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.heave() = msg.get()->data;	
		publish_pwm();
	}

	void yaw_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.yaw() = msg.get()->data;	
		publish_pwm();
	}

	void pitch_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.pitch() = msg.get()->data;	
		publish_pwm();	
	}

	void roll_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.roll() = msg.get()->data;	
		publish_pwm();
	}

	void publish_pwm() {
		auto thrusts = thruster_manager.calculate_thrusts(screw);
		auto pwms = thruster_manager.thrusts_to_pwms(thrusts);

	}
};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "thruster");
	ThrusterNode thruster_node;
	thruster_node.run();

	return 0;
}