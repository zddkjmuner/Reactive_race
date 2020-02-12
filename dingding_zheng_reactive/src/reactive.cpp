#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

//#include <race/drive_param.h>


class Reactive {

private:
    ros::NodeHandle n;
    ros::Subscriber scan;
    ros::Publisher drive;

public:
    Reactive() {
	n = ros::NodeHandle();
	drive = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav",10);
	scan = n.subscribe("/scan", 1, &Reactive::scan_callback, this);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
	//get the range from callback
	double mina, maxa, angleinc;
	int sample_time;
	mina = scan_msg -> angle_min;
	maxa = scan_msg -> angle_max;
	angleinc = scan_msg -> angle_increment;
	sample_time = int((maxa-mina)/angleinc);
	std::vector<float> distant;
	distant = scan_msg -> ranges;


	//filter to get the range and corresponging degrees
	std::vector<std::tuple<double,double>> range(420);
	std::tuple<double,double> degree_pair;
	double degree = 0;
	for(int i=0;i<distant.size();i++){
	    if(std::isinf(distant[i])){
		distant[i] = 60;
	    }
	    if(i>329 && i<750){
		if(distant[i]>3){
		    distant[i] = 3;
		}
		if(std::isnan(distant[i])){
		    continue;
	    	}
		if(i<=540){
		    degree = -(540-i)/3.0;
		    degree_pair = std::make_pair(distant[i],degree);
		    range[i-330] = degree_pair;
		}else{
		    degree = (i-540)/3.0;
		    degree_pair = std::make_pair(distant[i],degree);
		    range[i-330] = degree_pair;
		}
	    }
	}
	//ROS_INFO("the size of range %d", range.size());
	double d = 0;
	for(int i=0;i<range.size();i++){
	    //d = std::get<1>(range[i]);
	    //ROS_INFO("the degree %d is about %f", i, d);
	    //ROS_INFO("the %d distant is about %f", i, std::get<0>(range[i]));
	}
	
	//create the safety bubbles
	int bubble_range = 220;//30 degrees
	double min = 100;
	int min_index;
	for(int i=0;i<range.size();i++){
	    if(std::get<0>(range[i])<min){
		min = std::get<0>(range[i]);
		min_index = i;
	    }
	}	
	ROS_INFO("the final min %d is %f", min_index, min);
	for(int i=0;i<range.size();i++){
	    if(i>min_index-bubble_range/2 && i<min_index+bubble_range/2){
		std::get<0>(range[i]) = 0.0;
	    }
	}

	//find the disparity and set to obstacles
	std::vector<double> disparity(range.size());
	double d_threshold = 0.8;
	for(int i=0;i<range.size()-1;i++){
	    disparity[i] = std::get<0>(range[i+1])-std::get<0>(range[i]);
	    //ROS_INFO("the %d distant is about %f", i, std::get<0>(range[i]));
	    //ROS_INFO("the disparity %f", disparity[i]);
	}
	disparity[range.size()] = 0;
	bool obs = false;
	std::vector<int> obstacles;
	obstacles.push_back(0);
	for(int i=0;i<range.size();i++){
	    if(disparity[i]<-d_threshold){
		//obs = true;
		obstacles.push_back(i);
	    }else if(disparity[i]>d_threshold){
		//obs = false;
		obstacles.push_back(i);
	    }else{
	    }
	}
	obstacles.push_back(420);
	//now we have the disparity vector

	std::vector<double> average(obstacles.size()-1);
	for(int i=0;i<obstacles.size()-1;i++){
	    for(int j=obstacles[i];j<obstacles[i+1];j++){
		average[i] += std::get<0>(range[j]);
	    }
	    average[i] = average[i]/(obstacles[i+1]-obstacles[i]);
	    ROS_INFO("the %d th average is : %f ", i, average[i]);
	}
	double max_ave = 0;
	int range_index = 0;
	for(int i=0;i<obstacles.size();i++){
	    if(average[i]>max_ave){
		max_ave = average[i];
		range_index = i;
	    }
	}

	//the goal point, take the middle point
	int mid = (obstacles[range_index+1]-obstacles[range_index])/2;
	//double max_degree = std::get<1>(range[mid]);

	//find the best point in the max group
	double max_dist = 0;
	int max_index = 0;
	double max_degree;
	bool first = true;
	int first_index = 1000;
	int cnt3 = 0;
	for(int i=0;i<range.size();i++){
	    if(std::get<0>(range[i])>=max_dist && i>=obstacles[range_index] && i<=obstacles[range_index+1]){
		max_dist = std::get<0>(range[i]);
		max_index = i;
		max_degree = std::get<1>(range[i]);
		if(max_dist ==3 && first){
		    first = false;
		    first_index = i;
		}
		if(max_dist ==3){
		    cnt3 ++;
		}
	    ROS_INFO("%d th inside : %f ", i, std::get<0>(range[i]));
	    }
	}
	if(first_index != 1000){
	    max_dist = 3;
	    max_index = first_index + cnt3/2;
	    max_degree = std::get<1>(range[max_index]);
	}

	//
	if(max_index-mid > 30 && max_degree>25){
	    max_index -= 60;
	    max_degree = std::get<1>(range[max_index]);
	}

	ROS_INFO("now the farthest is the %d th and the distant is %f of %f degrees", max_index, max_dist, max_degree);
	
	ROS_INFO("the final degree is : %f ", max_degree);

/*
	int max_count = 0;
	int range_index;
	for(int i=0;i<obstacles.size();i=i+2){
	    if(obstacles[i+1]-obstacles[i]>max_count){
		max_count = obstacles[i+1]-obstacles[i];
		range_index = i;
	    }
	    if(obstacles.size()%2!=0 && i>=obstacles.size()-2){
		break;
	    }
	}
	ROS_INFO("range %d is %d", range_index, max_count);//i for max group
	for(int i=0;i<obstacles.size();i=i+1){
	    ROS_INFO("range %d is %d", i, obstacles[i]); 
	}

	*/

	//calculate the angle
	double angle = max_degree/180*M_PI;
	if(min<0.5){
	    angle*=1.5;
	}
	if(angle>0){
	    angle*=0.6;
	}

	//publish the drive message
	double velocity = 3.5;
	Reactive::navigation(angle, velocity);

    }

    void navigation(double angle, double velocity){
	//ROS_INFO("Drive at %f angle with %f speed", angle, velocity); 
   	   
	ackermann_msgs::AckermannDriveStamped drv;
	drv.header.frame_id = "laser";
	drv.drive.steering_angle = angle;
	drv.drive.speed = velocity;
	drive.publish(drv);
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "reactive");
    Reactive re;
    ros::spin();
    return 0;
}
