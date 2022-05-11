#include <ESW_task/random_number_publisher.h>

#include <glog/logging.h>

int main(int argc, char **argv) {

	int rate;
	std_msgs::Int32 random_number;

	google::InitGoogleLogging(argv[0]);

	ros::init(argc, argv, "random_number_publisher_node");
	ros::NodeHandle n, private_node_handle_("~");

	private_node_handle_.param("rate", rate, int(10));

	ros::Publisher random_number_pub = n.advertise<std_msgs::Int32>("random_number", 1);

	srand(time(NULL));

	ros::Rate loop_rate(rate);

	while(ros::ok()) {

		//generating random number in range [1, 1000000]
		random_number.data = rand() % 1000000 + 1;

		//publishing generated number on ros topic
		random_number_pub.publish(random_number);

		//loging data using glog
		LOG(INFO) << "Random number: " << random_number;

		//sleeping for 1/rate seconds
		loop_rate.sleep();
	}

	google::ShutdownGoogleLogging();

	return 0;
}