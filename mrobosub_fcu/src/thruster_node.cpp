#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <ros/console.h>
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

		XmlRpc::XmlRpcValue fit_params, thruster_params, voltage_param, thrusters_type_params;
		ros::param::get("~/fits", fit_params);
		ros::param::get("~/voltage", voltage_param);
		ros::param::get("~/thrusters", thruster_params);
		ros::param::get("~/thruster_types", thrusters_type_params);
		cout << voltage_param.getType() << endl;
		//assert(voltage_param.getType() == XmlRpcValue::TypeDouble);
		voltage = double(voltage_param);
		auto fits = parse_fits_param(fit_params);
		auto thrusters = parse_thrusters_param(thruster_params, thrusters_type_params, voltage, fits);

		thruster_manager = ThrusterManager{thrusters};
		// ros::Rate loop_rate(10);

		ros::spin();
	}

private:
	ros::NodeHandle n;
	ros::Publisher motor_pub = n.advertise<std_msgs::UInt16MultiArray>("motor", 10);
	ros::Publisher thrusts_pub = n.advertise<std_msgs::Float64MultiArray>("thrusts",10);
	ThrusterManager thruster_manager;
	double voltage;
	Screw<double> screw{0,0,0,0,0,0};

	static Thruster::QuadParams parse_quad_param(const XmlRpc::XmlRpcValue &value) {
		return Thruster::QuadParams(
			double(value["a"]),
			double(value["b"]),
			double(value["c"])
		);
	}

	static Screw<double> parse_pose_param(const XmlRpc::XmlRpcValue &value){
		return Screw<double>{
			double(value["surge"]),
			double(value["sway"]),
			double(value["heave"]),
			double(value["roll"]),
			double(value["pitch"]),
			double(value["yaw"]),
		};
	}

	static vector<Thruster> parse_thrusters_param(
		const XmlRpc::XmlRpcValue &value, const XmlRpc::XmlRpcValue &types_value,
		double voltage, const vector<Thruster::PWMFit> &fits
	) {
		vector<Thruster> thrusters;

		try{
			assert(value.getType() == XmlRpcValue::Type::TypeArray);
			
			for(size_t i = 0; i < value.size(); ++i){
				ROS_DEBUG_STREAM(i);
				auto &fit_param = value[i];
				auto &thruster_type = fit_param["type"];
				auto &thruster_defaults = types_value[string(thruster_type)];

				Screw<double> pose = parse_pose_param(fit_param["pose"]);
				
				thrusters.emplace_back(
					int(fit_param["id"]),
					pose,
					int(thruster_defaults["epsilon"]),
					int(thruster_defaults["zero-pwm"]),
					int(thruster_defaults["min-neg-pwm"]),
					int(thruster_defaults["min-pos-pwm"]),
					int(thruster_defaults["max-neg-pwm"]),
					int(thruster_defaults["max-pos-pwm"]),
					bool(fit_param["reversed"]),
					fits,
					double(fit_param["drag"])
				);
			}
		} catch(const XmlRpcException &e) {
			cerr << "[ERROR] Param parse: " << e.getMessage() << endl;
		}

		return thrusters;
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
		publish_thrusts();
	}

	void sway_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.sway() = msg.get()->data;
		publish_pwm();
		publish_thrusts();
	}

	void heave_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.heave() = msg.get()->data;	
		publish_pwm();
		publish_thrusts();
	}

	void yaw_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.yaw() = msg.get()->data;	
		publish_pwm();
		publish_thrusts();
	}

	void pitch_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.pitch() = msg.get()->data;	
		publish_pwm();
		publish_thrusts();
	}

	void roll_callback(const std_msgs::Float64::ConstPtr& msg) {
		screw.roll() = msg.get()->data;	
		publish_pwm();
		publish_thrusts();
	}

	void publish_pwm() {
		auto thrusts = thruster_manager.calculate_thrusts(screw);
		auto pwms = thruster_manager.thrusts_to_pwms(thrusts, voltage);

		auto to_publish = std_msgs::UInt16MultiArray();
		to_publish.data = std::move(pwms);
		to_publish.layout.dim.push_back(std_msgs::MultiArrayDimension());  
		to_publish.layout.dim[0].size = pwms.size();
		to_publish.layout.dim[0].stride = 1;
		motor_pub.publish(to_publish);
	}
	
	void publish_thrusts(){
		auto thrustsEigen = thruster_manager.calculate_thrusts(screw);
		vector<double> thrusts(thrustsEigen.rows());
		for(size_t i = 0; i < thrustsEigen.rows(); ++i) {
       		thrusts[i] = thrustsEigen(i);
    	}

		auto to_publish = std_msgs::Float64MultiArray();
		to_publish.data = std::move(thrusts);
		to_publish.layout.dim.push_back(std_msgs::MultiArrayDimension());  
		to_publish.layout.dim[0].size = thrusts.size();
		to_publish.layout.dim[0].stride = 1;
		thrusts_pub.publish(to_publish);
	}

};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "thruster");
	ThrusterNode thruster_node;
	thruster_node.run();

	return 0;
}