#include <pluginlib/class_list_macros.hpp>
#include "pcl_ros/filters/frame_skipper.h"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FrameSkipper::onInit ()
{
  // Call the super onInit ()
  PCLNodelet::onInit ();

  // Enable the dynamic reconfigure service
  srv_ = boost::make_shared <dynamic_reconfigure::Server<FrameSkipperConfig> > (*pnh_);
  dynamic_reconfigure::Server<FrameSkipperConfig>::CallbackType f = boost::bind (&FrameSkipper::config_callback, this, _1, _2);
  srv_->setCallback (f);

  // Advertise the output topics
  pub_output_ = advertise<PointCloud2> (*pnh_, "output", max_queue_size_);

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FrameSkipper::subscribe ()
{
  sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
  sub_input_filter_.registerCallback (bind (&FrameSkipper::filter, this, _1));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FrameSkipper::unsubscribe ()
{
  sub_input_filter_.unsubscribe ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::FrameSkipper::config_callback (pcl_ros::FrameSkipperConfig &config, uint32_t /*level*/)
{
  skip = config.skip;
  NODELET_DEBUG ("[config_callback] Setting skip to: %u.", skip);
}

void
pcl_ros::FrameSkipper::filter (const PointCloud2::ConstPtr &cloud)
{
  if( i % (skip + 1) == 0) {
    pub_output_.publish (cloud);
  }
  
  i++;
}

typedef pcl_ros::FrameSkipper FrameSkipper;
PLUGINLIB_EXPORT_CLASS(FrameSkipper,nodelet::Nodelet)

