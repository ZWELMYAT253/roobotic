#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <chefbot_bringup/rpm.h>
#include <chefbot_bringup/spray.h>
#define wheel_diameter  0.069   
#define track_width     0.210   
#define pi              3.1415926

float lin_smooth = 0.5; //1.0;
float ang_smooth = 1.0;

ros::Publisher pub;
ros::Publisher pub2;
chefbot_bringup::rpm RPM;
chefbot_bringup::spray SPRAY;

float L = track_width / 2.0;
float ms_to_rpm_ratio = 60.0/ (pi * wheel_diameter);


void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
	// some clacluation
		int spray_btn = joy->buttons[1];
		int other_btn = joy->buttons[2];

		double x = joy->axes[1];
		double theta = ang_smooth * joy->axes[3];
		//double theta = joy->axes[3];

		double right = 1.0 * x + (theta * L);  // ms
		double left  = 1.0 * x - (theta * L); // ms

	int	rpm_right = right * ms_to_rpm_ratio;  //rpm
	int	rpm_left  = left  * ms_to_rpm_ratio;  //rpm

	rpm_right *= lin_smooth;
	rpm_left  *= lin_smooth;

	RPM.desire_rpm_right = rpm_right;
	RPM.desire_rpm_left  = rpm_left;
	pub.publish(RPM);

	SPRAY.spray = spray_btn;
	SPRAY.other = other_btn;
	pub2.publish(SPRAY);
}
int main(int argc, char **argv)
{
	ros::init(argc,argv, "joy_vel");
	ros::NodeHandle nh;

	pub  = nh.advertise<chefbot_bringup::rpm>("/desire_rpm_joy",50);
	pub2 = nh.advertise<chefbot_bringup::spray>("/desire_spray",50);
	ros::Subscriber sub = nh.subscribe("/joy",50, joyCallback);


	ros::spin();
	return 0;
}