#ifndef PCL_ROS_FILTERS_FRAME_SKIPPER_H_
#define PCL_ROS_FILTERS_FRAME_SKIPPER_H_

#include "pcl_ros/pcl_nodelet.h"
#include <message_filters/pass_through.h>

// PCL includes
#include "pcl_ros/filters/filter.h"

// Dynamic reconfigure
#include "pcl_ros/FrameSkipperConfig.h"

namespace pcl_ros
{
  namespace sync_policies = message_filters::sync_policies;

  class FrameSkipper : public PCLNodelet
  {
    public:
      typedef sensor_msgs::PointCloud2 PointCloud2;
      typedef PointCloud2::Ptr PointCloud2Ptr;
      typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    protected:
      /** \brief Pointer to a dynamic reconfigure service. */
      boost::shared_ptr <dynamic_reconfigure::Server<pcl_ros::FrameSkipperConfig> > srv_;

      /** \brief The message filter subscriber for PointCloud2. */
      message_filters::Subscriber<PointCloud2> sub_input_filter_;

      /** \brief The output PointIndices publisher. */
      ros::Publisher pub_output_;

      /** \brief Nodelet initialization routine. */
      void onInit ();

      /** \brief LazyNodelet connection routine. */
      void subscribe ();
      void unsubscribe ();

      /** \brief Dynamic reconfigure callback
        * \param config the config object
        * \param level the dynamic reconfigure level
        */
      void 
      config_callback (pcl_ros::FrameSkipperConfig &config, uint32_t level);

      /** \brief Input point cloud callback.
        * \param cloud the pointer to the input point cloud
        */
      void filter (const PointCloud2::ConstPtr &cloud);

      int skip = 0;
      int i = 0;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef PCL_ROS_FILTERS_FRAME_SKIPPER_H_
