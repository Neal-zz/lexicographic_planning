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

    // 判断 (x,y) 是否与环境碰撞
    bool isIncollision(float x, float y);

    // 计算两点之间的距离
    float distance(state_t* state_from, state_t* state_to);

    // 利用 black region and grey region 返回 c1 cost。
    float getCloseCollisionCost(float x, float y);

    // 计算选择当前 edge 的 costs
    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]);

    // 查找 cost 最小的路标节点
    state_t* minCostStateInQueue(vector<state_t*> Queue);

    // 更新 toState 的 costs
    void updateCosts(state_t* fromState, state_t* toState, int neighborInd);

    // 循环查找父节点，连接为 pathOut
    nav_msgs::Path extractPath(state_t* stateCur);

    // 建立邻接矩阵，16 行 n 列。储存每个节点的 x,y,theta。
    void buildAdjacencyMatrix();

    // 连接邻接矩阵，建立一张 PRM 有向搜索图。换言之，计算每个节点可去的 8 个邻接节点的 costs.
    void connectAdjacencyMatrix();

    // 搜索找到一条 cost 最小的局部路径
    bool searchAdjacencyMatrix();
    
    // 路标节点与路径可视化
    void visualization();

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
    ros::Publisher pubPRMGraph;
    ros::Publisher pubExecutePath;
    RolloutGeneratorNS::RolloutGenerator openPlannerRollOut;
    std::vector<state_t*> nodeList;
    std::vector<vector<state_t*> > adjacencyMatrix;
    nav_msgs::Path executePath;
    int adjacency_width_grid;
    int adjacency_length_grid;
    bool path_generated;

};