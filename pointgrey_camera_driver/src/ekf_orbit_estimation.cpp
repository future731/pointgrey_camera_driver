#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <geometry_msgs/PointStamped.h>

namespace pointgrey_camera_driver
{
class PointGreyBallOrbitEstimationNodelet : public nodelet::Nodelet
{
public:
  // Constructor
  PointGreyBallOrbitEstimationNodelet() = default;

  ~PointGreyBallOrbitEstimationNodelet()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    if (pub_thread_)
    {
      pub_thread_->interrupt();
      pub_thread_->join();
    }
  }

private:
  void connectCb()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    if (pub_orbit_.getNumSubscribers() == 0)
    {
      if (pub_thread_)
      {
        pub_thread_->interrupt();
        scoped_lock.unlock();
        pub_thread_->join();
        scoped_lock.lock();
        pub_thread_.reset();
      }
    }
    else if (not pub_thread_)
    {
      pub_thread_.reset(new boost::thread(boost::bind(&PointGreyBallOrbitEstimationNodelet::loop, this)));
    }
  }

  virtual void onInit()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    ros::SubscriberStatusCallback cb = boost::bind(&PointGreyBallOrbitEstimationNodelet::connectCb, this);
    pub_orbit_ = getMTNodeHandle().advertise<geometry_msgs::PointStamped>("/pointgrey/ball_orbit", 1, cb, cb);
  }

  void callback(const boost::shared_ptr<geometry_msgs::PointStamped const>& point)
  {
  }

  void loop()
  {
    sub_point_ = getMTNodeHandle().subscribe<geometry_msgs::PointStamped>(
        "/pointgrey/ball_point", 1, boost::bind(&PointGreyBallOrbitEstimationNodelet::callback, this, _1));
    pub_thread_->yield();
    boost::this_thread::disable_interruption no_interruption{};
    ros::Rate loop2(1);
    while (not boost::this_thread::interruption_requested())
    {
      if (pub_orbit_.getNumSubscribers() == 0)
      {
        break;
      }
      loop2.sleep();
    }
  }

  geometry_msgs::PointStamped p_msg_;
  ros::Subscriber sub_point_;
  ros::Publisher pub_orbit_;
  boost::shared_ptr<boost::thread> pub_thread_;
  boost::mutex connect_mutex_;
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreyBallOrbitEstimationNodelet,
                        pointgrey_camera_driver::PointGreyBallOrbitEstimationNodelet, nodelet::Nodelet);
}
