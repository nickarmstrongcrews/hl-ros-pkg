#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvideo.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_camera_node");
    ros::NodeHandle nh("~");

     int fps;
     nh.param("fps", fps, 5);
//     if(fps != 5 && fps != 10 && fps != 15 && fps != 30) fps = 5;


    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/nao/camera", 1); // only queue most recent frame

    // Connecting to video module
    AL::ALPtr<AL::ALVideoDeviceProxy> camera(new AL::ALVideoDeviceProxy("127.0.0.1", 9559));

    // Registering a generic video module
    std::string gvmName = camera->subscribe(
        "naoqi_wrapper",
        // kVGA   ( 640 * 480 ),
        // kQVGA  ( 320 * 240 ),
        // kQQVGA ( 160 * 120 ).
        AL::kQQVGA,
        // kYuvColorSpace,
        // kYUVColorSpace,
        // kYUV422InterlacedColorSpace,
        // kRGBColorSpace,
        AL::kYuvColorSpace,
        // vision module required among : 5, 10, 15, and 30 fps.
        // (AL note: this field has no effect right now but will be implemented
        // in a future version)
        fps
    );

    ros::Rate loop_rate(fps);

    while (nh.ok()) {

        AL::ALValue alimage = camera->getImageRemote(gvmName);
        // TODO release needed with remote?
        camera->releaseImage(gvmName);

        sensor_msgs::Image msg;
        // TODO use time given by AL
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/CameraTop_frame";

        msg.width  = (int) alimage[0];
        msg.height = (int) alimage[1];
        //msg.encoding = "rgb8"; // ok for kRGBColorSpace
        msg.encoding = "mono8"; // ok for AL::kYuvColorSpace
        msg.step = msg.width * ((int) alimage[2]);
        msg.data = alimage[6];

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    camera->unsubscribe(gvmName);
}
