#include <ESW_task/random_number_subscriber.h>

#include <glog/logging.h>
#include <stdio.h>
#include <math.h>
#include <cmath>

RandomNumberSubscriber::RandomNumberSubscriber(void) :
	random_number_(0),
	is_new_random_number_(false) {}

//ros callback which receives random numbers from topic /random_number
void RandomNumberSubscriber::randomNumberCb(const std_msgs::Int32 &msg) {
	random_number_ = msg.data;
	is_new_random_number_ = true;
}

//This method returns random number which is received trought ros topic
int RandomNumberSubscriber::getRandomNumber(void) {
	is_new_random_number_ = false;
	return random_number_;
}


//If the stored random number is unprocessed this method will return true
bool RandomNumberSubscriber::isNewRandomNumber(void) {
	return is_new_random_number_;
}


//This method returns total number of prime numbers in range of [1, upper_limit]
int getNumberOfPrimeNumbersInRange(int upper_limit)
{
    int i, j, flag, counter = 0;
 
    for (i = 2; i < upper_limit; i++) {
 		flag = 1;

        for (j = 2; j <= sqrt(i); ++j) {
            if (i % j == 0) {
                flag = 0;
                break;
            }
        }
 
        if (flag == 1)
            counter++;
    }

    return counter;
}

int main(int argc, char **argv) {
	int rate, random_number, square_num;
	int number_of_prime_numbers_in_range;
	float root_num, fract_part, whole_part;
	bool perferct_square_flag = false;
	ESW_task::ESWTask esw_msg;


	google::InitGoogleLogging(argv[0]);

	ros::init(argc, argv, "random_number_subscriber_node");
	ros::NodeHandle n, private_node_handle_("~");

	private_node_handle_.param("rate", rate, int(20));

	RandomNumberSubscriber random_number_subscriber;

	ros::Subscriber clock_ros_sub = n.subscribe("random_number", 10, &RandomNumberSubscriber::randomNumberCb, &random_number_subscriber);

	ros::Publisher square_and_prime_number_pub = n.advertise<ESW_task::ESWTask>("output", 1);

	ros::Rate loop_rate(rate);

	while (ros::ok()) {
		ros::spinOnce();

		//here we check if new msg has arrived. we only want to process numbers that has not be processed. 
		if (random_number_subscriber.isNewRandomNumber()) {
			random_number = random_number_subscriber.getRandomNumber();

			//we calculated square root of the given number and we take only his int part. 
			//If the square of the int part is equal to the random number then this random number is perfect square. 
			root_num = sqrt(random_number);
			fract_part = std::modf(root_num, &whole_part);
			square_num = whole_part * whole_part;

			esw_msg.header.stamp = ros::Time::now();

			if (square_num == random_number) {
				//here we use function to calculate number of prime numbers in range of [1, random_number]
				//we publish this data to the topic with custom msg. 
				perferct_square_flag = true;
				number_of_prime_numbers_in_range = getNumberOfPrimeNumbersInRange(random_number);
				LOG(INFO) << "Random number: " << random_number;
				LOG(INFO) << "Perfect square: 1";
				LOG(INFO) << "Number of prime numbers: " << number_of_prime_numbers_in_range;

				esw_msg.number = random_number;
				esw_msg.number_of_prime_numbers = number_of_prime_numbers_in_range;
				esw_msg.is_perfect_square = perferct_square_flag;
			}
			else
			{
				perferct_square_flag = false;
				LOG(INFO) << "Random number " << random_number <<" is not perfect square.";

				esw_msg.number = random_number;
				esw_msg.number_of_prime_numbers = 0;
				esw_msg.is_perfect_square = perferct_square_flag;
			}

			square_and_prime_number_pub.publish(esw_msg);

		}

		loop_rate.sleep();
	}

	google::ShutdownGoogleLogging();

	return 0;
}