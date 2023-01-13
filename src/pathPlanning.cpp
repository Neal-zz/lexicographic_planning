#include "pathPlanning.h"

pathPlanning::pathPlanning() {
    
    /* 用于发布地图的变量*/
    pubSurroundMapCloudDS = nh.advertise<sensor_msgs::PointCloud2>("planning/obstacle/surround_cloud_map_downsample", 1);
    pubOccupancyMap2 = nh.advertise<nav_msgs::OccupancyGrid> ("planning/obstacle/map_inflated", 1);
    downSizeFilter.setLeafSize(_mapResolution, _mapResolution, _mapResolution);
    pc_published = false;
    initializeOccupancyMap();

    /* 用于发布路径的变量*/
    pubPathSmooth = nh.advertise<nav_msgs::Path> ("planning/server/path_blueprint_smooth", 1);
    path_published = false;

    /* 用于生成路径的变量*/
    path_generated = false;
}

void pathPlanning::initializeOccupancyMap() {

    occupancyMap2D.header.frame_id = "map";
    occupancyMap2D.info.width = _local_map_grid_num;   // 400
    occupancyMap2D.info.height = _local_map_grid_num;  // 400
    occupancyMap2D.info.resolution = _mapResolution;   // 0.05
    occupancyMap2D.info.origin.orientation.x = 0.0;
    occupancyMap2D.info.origin.orientation.y = 0.0;
    occupancyMap2D.info.origin.orientation.z = 0.0;
    occupancyMap2D.info.origin.orientation.w = 1.0;
    occupancyMap2D.data.resize(occupancyMap2D.info.width * occupancyMap2D.info.height);
}

void pathPlanning::publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame) {
    if (thisPub->getNumSubscribers() == 0) {
        return;
    }
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud);
}

void pathPlanning::cloudHandler() {

    if (pc_published) {
        return;
    }

    pcl::PLYReader reader;
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
    reader.read(string(string(ROOT_DIR) + "PCD/") + "data.ply", *laserCloudIn);
    laserCloudIn->header.frame_id = "base_link";
    laserCloudIn->header.stamp = 0;

    // down-sample cloud
    pcl::PointCloud<PointType>::Ptr laserCloudInDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setInputCloud(laserCloudIn);
    downSizeFilter.filter(*laserCloudInDS);

    // filter out the floor and the ceiling
    pcl::PointCloud<PointType>::Ptr laserCloudFiltered(new pcl::PointCloud<PointType>());
    for (int i = 0; i < laserCloudInDS->size(); ++i) {
        PointType p;
        p.x = laserCloudInDS->points[i].x;
        p.y = laserCloudInDS->points[i].y;
        p.z = laserCloudInDS->points[i].z;
        if (p.z > 2.2 || p.z < 0.2) {
            continue;
        }
        laserCloudFiltered->push_back(p);
    }

    PointType robotPoint;
    robotPoint.x = 0.0;
    robotPoint.y = 0.0;
    robotPoint.z = 0.0;
    // update occupancy map
    if (pubOccupancyMap2.getNumSubscribers() != 0) {
        // 图中红色部分
        std_msgs::Header cloudHeader;
        publishCloud(&pubSurroundMapCloudDS, laserCloudFiltered, cloudHeader.stamp, "map");

        occupancyMap2D.header.stamp = cloudHeader.stamp;
        occupancyMap2D.info.origin.position.x = robotPoint.x - _local_map_length / 2.0;
        occupancyMap2D.info.origin.position.y = robotPoint.y - _local_map_length / 2.0;
        occupancyMap2D.info.origin.position.z = -0.1;
        std::fill(occupancyMap2D.data.begin(), occupancyMap2D.data.end(), 0);

        int search_num_ob = round(_occuMapInflation / _mapResolution);  // 0.2/0.05
        for (int i = 0; i < laserCloudFiltered->size(); ++i) {
            int index_x = (laserCloudFiltered->points[i].x - occupancyMap2D.info.origin.position.x) / _mapResolution;
            int index_y = (laserCloudFiltered->points[i].y - occupancyMap2D.info.origin.position.y) / _mapResolution;

            for (int m = -search_num_ob; m <= search_num_ob; ++m) {
                for (int n = -search_num_ob; n <= search_num_ob; ++n) {
                    if (sqrt(float(m*m + n*n)) * _mapResolution > _occuMapInflation) {
                        continue;
                    }

                    int x_id = index_x + m;
                    int y_id = index_y + n;
                    if (x_id < 0 || y_id < 0 || x_id >= _local_map_grid_num || y_id >= _local_map_grid_num) {
                        continue;
                    }
                    // update the black region, taking into account the robot's size.
                    int index = y_id * occupancyMap2D.info.width + x_id;
                    occupancyMap2D.data[index] = 100;
                }
            }
        }

        // inflate occupancy map
        int search_num_field = round(_occuMapField / _mapResolution);  // 0.2/0.05
        float scale = 100.0 / max((float)log(1.0 / _mapResolution), (float)1e-6);
        for (int i = 0; i < occupancyMap2D.info.width; ++i) {       // 400
            for (int j = 0; j < occupancyMap2D.info.height; ++j) {  // 400
                // update the gray region around the black region, using the inverse distance: R(x).
                if (occupancyMap2D.data[j * occupancyMap2D.info.width + i] == 100) {
                    for (int m = -search_num_field; m <= search_num_field; ++m) {
                        for (int n = -search_num_field; n <= search_num_field; ++n) {
                            float dist = sqrt(float(m*m + n*n)) * _mapResolution;
                            if (dist > _occuMapField) {
                                continue;
                            }

                            int newIdX = i + m;
                            int newIdY = j + n;
                            if (newIdX < 0 || newIdX >= occupancyMap2D.info.width ||
                                newIdY < 0 || newIdY >= occupancyMap2D.info.height) {
                                continue;
                            }

                            int index = newIdX + newIdY * occupancyMap2D.info.width;
                            if (occupancyMap2D.data[index] != 100) {
                                float inverse_dist = 1.0 / max(dist, _mapResolution);
                                float log_inverse_dist = max((float)log(inverse_dist), (float)1e-6);
                                int8_t grid_value = min(int(log_inverse_dist * scale), 99);
                                occupancyMap2D.data[index] = max(occupancyMap2D.data[index], grid_value);
                            }
                        }
                    }
                }
            }
        }
        pubOccupancyMap2.publish(occupancyMap2D);
        pc_published = true;
    }
}

geometry_msgs::PoseStamped pathPlanning::createPoseStamped(float x, float y, float z) const {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y; 
    pose.pose.position.z = z;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    return pose;
}

void pathPlanning::pathHandler() {

    if (path_published) {
        return;
    }

    // create raw path
    nav_msgs::Path pathRaw = nav_msgs::Path();
    pathRaw.poses.push_back(createPoseStamped(-3, -4, 0));
    pathRaw.poses.push_back(createPoseStamped(4, 2, 0));
    // smooth path
    pathSmooth = processPath(pathRaw);

    // publish smooth path
    if (pubPathSmooth.getNumSubscribers() != 0) {
        pathSmooth.header.frame_id = "map";
        pathSmooth.header.stamp = ros::Time::now();
        pubPathSmooth.publish(pathSmooth);
        path_published = true;
    }
}

void pathPlanning::buildAdjacencyMatrix() {

    // 返回 globalPath, centerPath（局部路径）, remainingPath（局部路径之后的部分）, alternativePaths（rollouts）
    openPlannerRollOut.run(transform, globalPathMessage, globalPath, centerPath, remainingPath, alternativePaths);

    // clear memory
    for (int i = 0; i < nodeList.size(); ++i) {
        state_t *stateCur = nodeList[i];
        delete stateCur;
    }
    nodeList.clear();

    // allocate vector size
    adjacency_width_grid = alternativePaths.size();  // 16 行
    adjacency_length_grid = alternativePaths[0].poses.size();  // 列
    adjacencyMatrix.resize(adjacency_width_grid);
    for (int i = 0; i < adjacency_width_grid; ++i) {
        adjacencyMatrix[i].resize(adjacency_length_grid);
    }

    // create new states for adjacency matrix
    for (int i = 0; i < alternativePaths.size(); ++i) {
        for (int j = 0; j < alternativePaths[i].poses.size(); ++j) {
            state_t *newState = new state_t;
            newState->x = alternativePaths[i].poses[j].pose.position.x;
            newState->y = alternativePaths[i].poses[j].pose.position.y;
            newState->z = 0;
            newState->theta = tf::getYaw(alternativePaths[i].poses[j].pose.orientation);
            newState->idx = i;
            newState->idy = j;
            newState->validFlag = true;
            newState->stateId = nodeList.size();

            nodeList.push_back(newState);
            adjacencyMatrix[i][j] = newState;
        }
    }
}

/* 穷举，找到一条 cost 最小的局部路径*/
void pathPlanning::generatePath() {

    std::lock_guard<std::mutex> lock(mtx);

    if (path_generated) {
        return;
    }

    if (path_published && pc_published) {
        // 建立邻接矩阵，16 行 n 列。储存每个节点的 x,y,theta
        buildAdjacencyMatrix();
        // 计算每个节点可去的 8 个邻接节点的 costs
        // connectAdjacencyMatrix();
        // // 搜索找到一条新的局部路径
        // searchAdjacencyMatrix();
        // // 可视化
        // visualization();
        // // 发布局部路径
        // publishPath();
        path_generation = true;
    }

}