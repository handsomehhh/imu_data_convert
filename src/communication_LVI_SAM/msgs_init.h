#ifndef _MSGS_INIT_H_
#define _MSGS_INIT_H_

#include "msgs_ros2struct.h"

void lio_sam_msgs_cloud_info_init(lio_sam_msgs_cloud_info& msgs)
{
    //只初始化自定义的变量
    msgs.startRingIndex_number = 0;
    msgs.endRingIndex_number = 0;
    msgs.pointRange_number = 0;
    msgs.pointColInd_number = 0;

    msgs.cloud_deskewed.fields_number = 0;
    msgs.cloud_deskewed.data_number = 0;
    msgs.cloud_corner.fields_number = 0;
    msgs.cloud_corner.data_number = 0;
    msgs.cloud_surface.fields_number = 0;
    msgs.cloud_surface.data_number = 0;

}

#endif