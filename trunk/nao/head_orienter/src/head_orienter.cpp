#include <ros/ros.h>
#include <nao_msgs/HeadAngles.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

ros::Publisher* pubPtr = NULL;

void callback(const geometry_msgs::PoseStampedConstPtr& msg) {

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);

  double y, p, r;
  btMatrix3x3 R;
  R.setRotation(q);
  R.getEulerYPR (y, p, r);

  nao_msgs::HeadAngles headAngleMsg;
  headAngleMsg.yaw = r; // swapped, to bring android phone frame into coincidence with robot head frame
  headAngleMsg.pitch = p;
  headAngleMsg.absolute = 1; // 1 means "absolute" position

  if(pubPtr != NULL) pubPtr->publish(headAngleMsg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "head_orienter");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nao_msgs::HeadAngles>("/head_angles", 1);
  pubPtr = &pub;

  ros::Subscriber sub = nh.subscribe("/android/orientation", 1, callback);

  ros::Rate r(10); // limit rate to 10 Hz

  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;

}
