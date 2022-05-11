#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include "ESW_task/ESWTask.h"

class RandomNumberSubscriber{
	private:
		int random_number_;
		bool is_new_random_number_;

	public:
		RandomNumberSubscriber(void);
		void randomNumberCb(const std_msgs::Int32 &msg);
		int getRandomNumber(void);
		bool isNewRandomNumber(void);
};