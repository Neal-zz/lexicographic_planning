/*
（1）把三维点云地图转换成为二维的栅格地图，并填充 black region and grey region, using the Risk(x)。
（2）发布全局路径。
*/
#pragma once
#include "utility.h"

class pathPlanning : public ParamServer {

public:
    pathPlanning();
    ~pathPlanning() {};
    
    // 初始化 occupancy map
    void initializeOccupancyMap();
    // 发布地图
    void cloudHandler();
    // 发布路径
    void pathHandler();
    // 生成路径
    void generatePath();


private:

    // 发布点云（红色区域）
    void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud,
        ros::Time thisStamp, std::string thisFrame);
    
    // 创建路径节点
    geometry_msgs::PoseStamped createPoseStamped(float x, float y, float z) const;

    // 建立邻接矩阵，16 行 n 列。储存每个节点的 x,y,theta。
    void buildAdjacencyMatrix();

    /* 用于发布地图的变量*/
    ros::Publisher pubSurroundMapCloudDS;  // 发布 pc
    ros::Publisher pubOccupancyMap2;       // 发布 black and grey region
    pcl::VoxelGrid<PointType> downSizeFilter;
    bool pc_published;
    nav_msgs::OccupancyGrid occupancyMap2D;

    /* 用于发布路径的变量*/
    ros::Publisher pubPathSmooth;
    nav_msgs::Path pathSmooth;
    bool path_published;
    
    /* 用于生成路径的变量*/
    std::mutex mtx;
    bool path_generated;

};