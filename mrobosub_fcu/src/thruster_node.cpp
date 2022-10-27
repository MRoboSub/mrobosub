#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include "std_msgs/String.h"

#include "ThrusterManager.hpp"
#include "Thruster.hpp"

#include <sstream>
#include <vector>
#include <map>
#include <cassert>
#include <iostream>

using namespace std;
using namespace XmlRpc;

Thruster::QuadParams parse_quad_param(const XmlRpc::XmlRpcValue &value) {
	return Thruster::QuadParams(
		double(value["a"]),
		double(value["b"]),
		double(value["c"])
	);
}

vector<Thruster::PWMFit> parse_fits_param(const XmlRpc::XmlRpcValue &value) {
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

int main(int argc, char **argv) {
	ros::init(argc, argv, "thruster");

	ros::NodeHandle n;

	// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {
		XmlRpc::XmlRpcValue fit_params;
		n.getParam("/fits", fit_params);
		auto fits = parse_fits_param(fit_params);

		// cout << fits[0].forward_quad_params.a << endl;

		// chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;
}