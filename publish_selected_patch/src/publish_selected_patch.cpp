#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"

#include "publish_selected_patch.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QVariant>
#include "std_msgs/String.h"
#include <sstream>
#include <algorithm> 
   
using namespace std;

namespace publish_selected_patch
{
PublishSelectedPatch::PublishSelectedPatch()
{
  updateTopic();
}

PublishSelectedPatch::~PublishSelectedPatch()
{
}

void PublishSelectedPatch::updateTopic()
{
  // nh_.param("frame_id", tf_frame_, std::string("/base_link"));
  cloud_topic_ = "/selected_patch";
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );
  // ROS_INFO( "Publishing data on topic %s with frame_id %s.",
  //           nh_.resolveName (cloud_topic_).c_str (),
  //           tf_frame_.c_str() );
}

int PublishSelectedPatch::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = rviz::SelectionTool::processMouseEvent( event );

  // determine current selection mode
  if( event.alt() )
  {
    selecting_ = false;
  }
  else
  {
    if( event.leftDown() )
    {
      selecting_ = true;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
    }
  }
  
  // We use get3DPoint( event.viewport, temp_x, temp_y, pos ) to replace the getSelection()
  // and pcl::PointCloud to construct the point cloud.

			
  if( selecting_ )
  {

	if( event.leftUp() )
	  {
	      vector<Ogre::Vector3> result_points;
	      int x_left = min(sel_start_x_, event.x);
	      int y_left = min(sel_start_y_, event.y);

	      int x_right = max(sel_start_x_, event.x);
	      int y_right = max(sel_start_y_, event.y);

	      Ogre::Vector3 pos_left;
	      Ogre::Vector3 pos_right;
	      bool success_l = context_->getSelectionManager()->get3DPoint(event.viewport, x_left, y_left, pos_left);
 	      bool success_r = context_->getSelectionManager()->get3DPoint(event.viewport, x_right, y_right, pos_right);

	      double pos_left_x = pos_left.x;
	      double pos_left_y = pos_left.y;
	      double pos_left_z = pos_left.z;

	      double pos_right_x = pos_right.x;
	      double pos_right_y = pos_right.y;
	      double pos_right_z = pos_right.z;


		std::string path = "/home/robot/Desktop/rmb-ml/test.txt";
		// writing to txt -> each element in a new line
		std::ofstream fout;
		fout.open(path.c_str());
		// left
		fout<<pos_left_x<<std::endl;
		fout<<pos_left_y<<std::endl;
		fout<<pos_left_z<<std::endl;

		// right
		fout<<pos_right_x<<std::endl;
		fout<<pos_right_y<<std::endl;
		fout<<pos_right_z<<std::endl;

		// info: in rviz the coordinates are swaped -> y_pos := z
		
	  }
  }
  
  return flags;
}

} // end namespace publish_selected_patch

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( publish_selected_patch::PublishSelectedPatch, rviz::Tool )
