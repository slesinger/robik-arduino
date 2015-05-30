#ifndef _ROS_nav_msgs_Odometry_h
#define _ROS_nav_msgs_Odometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace nav_msgs
{

  class Odometry : public ros::Msg
  {
    public:
      std_msgs::Header header;
      char * child_frame_id;
      geometry_msgs::Pose pose;
      geometry_msgs::Twist twist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t * length_child_frame_id = (uint32_t *)(outbuffer + offset);
      *length_child_frame_id = strlen( (const char*) this->child_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, *length_child_frame_id);
      offset += *length_child_frame_id;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_child_frame_id = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_child_frame_id-1]=0;
      this->child_frame_id = (char *)(inbuffer + offset-1);
      offset += length_child_frame_id;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dagny_msgs/OdometryLite"; };
    const char * getMD5(){ return "358519fefe994854542e1fa153efc73a"; };

  };

}
#endif
