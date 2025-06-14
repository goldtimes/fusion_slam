#pragma once

namespace slam::sensors {

enum class LIDAR_TYPE {
    AVIA = 0,
    MID360 = 1,
    LEISHEN16 = 2,
    ROBOSENSE16 = 3,
    Velodyne16 = 4,
    Velodyne32 = 5,
    Velodyne64 = 6,
    OUSTER128 = 7,
};

}