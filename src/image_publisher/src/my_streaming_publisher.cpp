#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{

  ROS_INFO("Streaming publisher; argc:%d   argv: %s   %s   %s    %s   %s\n",argc, argv[0], argv[1], argv[2], argv[3], argv[4]);

  if (argc < 5)
     ROS_ERROR("Arguments needed: camera_name video_source");
  else
  {
      std::string camera_name("camera_");
      camera_name.append(argv[1]);

      int video_source = atoi(argv[2]);
      ros::init(argc, argv, camera_name);

      ros::NodeHandle nh;
      image_transport::ImageTransport it(nh);
      std::string publisher_name(camera_name);
      publisher_name.append("/image");
      image_transport::Publisher pub = it.advertise(publisher_name, 1);

      cv::VideoCapture cap(video_source);
      // Check if video device can be opened with the given index
      if(!cap.isOpened()) return 1;
      cv::Mat frame;
      sensor_msgs::ImagePtr msg;

      ros::Rate loop_rate(20);
      while (nh.ok()) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
          pub.publish(msg);
          cv::waitKey(1);
        }

        ros::spinOnce();
        loop_rate.sleep();
      }
  }
}
