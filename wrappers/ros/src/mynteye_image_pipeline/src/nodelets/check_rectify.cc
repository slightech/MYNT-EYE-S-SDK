#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

namespace mynteye_image_pipeline{

  class CheckRectify : public nodelet::Nodelet{
    virtual ~CheckRectify(){}
    virtual void onInit(){
      ros::NodeHandle &pnh  = getPrivateNodeHandle();
      ros::NodeHandle &nh = getNodeHandle();

      //subscribe stereo rectify image
      std::string left_rect_topic  = "/tmp/left/image_rect"; 
      std::string right_rect_topic = "/tmp/right/image_rect";
      std::string pub_stereo_image_topic = "/view_image/stereo/image_rect";
      pnh.param("left_rect_topic", left_rect_topic, left_rect_topic);
      pnh.param("right_rect_topic", right_rect_topic, right_rect_topic);
      pnh.param("pub_stereo_image_topic", pub_stereo_image_topic, pub_stereo_image_topic);
      pnh.param("queue_size", _queue_size, _queue_size);
      pnh.param("approx_sync", _approx_sync, _approx_sync);
      {
        NODELET_INFO_STREAM("CheckRectify: left_rect_topic: "<< left_rect_topic); 
        NODELET_INFO_STREAM("CheckRectify: right_rect_topic: "<< right_rect_topic); 
        NODELET_INFO_STREAM("CheckRectify: pub_stereo_image_topic: "<< pub_stereo_image_topic); 
        NODELET_INFO_STREAM("CheckRectify: _queue_size: "<< _queue_size); 
        NODELET_INFO_STREAM("CheckRectify: _approx_sync: "<< _approx_sync); 
      }
      image_transport::ImageTransport it(nh); 
      _img_left_sub.subscribe(it, left_rect_topic, 1);
      _img_right_sub.subscribe(it, right_rect_topic, 1);
      stereo_pub.reset(new image_transport::Publisher(it.advertise(pub_stereo_image_topic, 1)));

      if (_approx_sync)
      {
        _approximate_sync.reset(new StereoApproSync(StereoApproximatePolicy(_queue_size), _img_left_sub,  _img_right_sub));  
        _approximate_sync->registerCallback(boost::bind(&CheckRectify::process, this, _1, _2));
      }
      else
      {
        _exact_sync.reset(new StereoExactSync(StereoExactPolicy(_queue_size), _img_left_sub,  _img_right_sub));  
        _exact_sync->registerCallback(boost::bind(&CheckRectify::process, this, _1, _2));
      }
    }

    private:
      
      void process(
          const sensor_msgs::ImageConstPtr &img_left_ptr,
          const sensor_msgs::ImageConstPtr &img_right_ptr
          ) {

          ROS_ASSERT(img_left_ptr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 && 
            img_right_ptr->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0
          );

          cv::Mat left_img = cv_bridge::toCvShare(img_left_ptr, "8UC1")->image;
          cv::Mat right_img = cv_bridge::toCvShare(img_right_ptr, "8UC1")->image;

	  cv::Mat stereo_img;
	  cv::hconcat(left_img, right_img, stereo_img);
	  for (int r = 0; r < stereo_img.rows; r += 16)
	  {
	    cv::line(stereo_img, cv::Point(0, r), cv::Point(stereo_img.cols, r), cv::Scalar(150, 150, 150), 1, 8);
	  }

          auto&& stereo_msg = cv_bridge::CvImage(img_left_ptr->header, sensor_msgs::image_encodings::MONO8, stereo_img);
	  stereo_pub->publish(stereo_msg.toImageMsg());
      }

    private:
      typedef image_transport::SubscriberFilter ImageSubscriber;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                              sensor_msgs::Image>
          StereoApproximatePolicy;
      typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
          StereoExactPolicy;
      typedef message_filters::Synchronizer<StereoApproximatePolicy>
          StereoApproSync;
      typedef message_filters::Synchronizer<StereoExactPolicy> StereoExactSync;

      ImageSubscriber _img_left_sub;
      ImageSubscriber _img_right_sub;
      boost::shared_ptr<StereoApproSync> _approximate_sync;
      boost::shared_ptr<StereoExactSync> _exact_sync;
      boost::shared_ptr<image_transport::Publisher> stereo_pub;
      int _queue_size = 5;
      bool _approx_sync = true;

  };
PLUGINLIB_EXPORT_CLASS(mynteye_image_pipeline::CheckRectify, nodelet::Nodelet);
}
