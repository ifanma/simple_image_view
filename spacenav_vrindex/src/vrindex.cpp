#include <ros/ros.h>
#include <nodelet/loader.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>

ros::Publisher pub;
int vrindex = 0;

int last_button0;
int last_button1;

void callback(const sensor_msgs::JoyConstPtr & msg)
{
    if (msg->buttons[0] == 1 && last_button0 == 0)
    {
        vrindex --;
    }
    else if (msg->buttons[1] == 1 && last_button1 == 0)
    {
        vrindex ++;
    }

    if (vrindex > 5)
    {
        vrindex = 5;
    }else if (vrindex < 0)
    {
        vrindex = 0;
    }

    std_msgs::Int8 a;
    a.data = int8_t(vrindex);

    if ((msg->buttons[0] == 1 && last_button0 == 0) || (msg->buttons[1] == 1 && last_button1 == 0))
    {
        pub.publish(a);
    }

    last_button0 = msg->buttons[0];
    last_button1 = msg->buttons[1];

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vrindex");
    ros::NodeHandle nh("~");
    pub = nh.advertise<std_msgs::Int8>("/vr_index", 10);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &callback);

    ros::spin();

    return 0;
}
