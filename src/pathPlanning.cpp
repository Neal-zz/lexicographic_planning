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
    pubPRMGraph = nh.advertise<visualization_msgs::MarkerArray>("planning/planning/prm_graph", 5);
    pubExecutePath = nh.advertise<nav_msgs::Path>("planning/planning/execute_path", 1);
    adjacency_width_grid = -1;
    adjacency_length_grid = -1;
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
        std::cout << "pc published" << std::endl;
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
        std::cout << "path published" << std::endl;
    }
}

bool pathPlanning::isIncollision(float x, float y) {

    int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
    int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
    int index = index_x + index_y * occupancyMap2D.info.width;
    if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
        index_y < 0 || index_y >= occupancyMap2D.info.height) {

        return false;
    }

    if (occupancyMap2D.data[index] == 100) {  // black region
        return true;
    }
    else {
        return false;
    }
}

float pathPlanning::distance(state_t* state_from, state_t* state_to) {

    return sqrt((state_to->x-state_from->x)*(state_to->x-state_from->x) + 
                (state_to->y-state_from->y)*(state_to->y-state_from->y) + 
                (state_to->z-state_from->z)*(state_to->z-state_from->z));
}

float pathPlanning::getCloseCollisionCost(float x, float y) {

    int index_x = (int)round((x - occupancyMap2D.info.origin.position.x) / _mapResolution);
    int index_y = (int)round((y - occupancyMap2D.info.origin.position.y) / _mapResolution);
    int index = index_x + index_y * occupancyMap2D.info.width;

    if (index_x < 0 || index_x >= occupancyMap2D.info.width ||
        index_y < 0 || index_y >= occupancyMap2D.info.height) {
        return 0;
    }
    if (occupancyMap2D.data[index] > 0) {
        return float(occupancyMap2D.data[index]);
    }
    else {
        return 0;
    }
}

bool pathPlanning::edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]) {
    
    // 0. initialize edgeCosts
    for (int i = 0; i < NUM_COSTS; ++i) {  // <4
        edgeCosts[i] = 0;
    }
    // 1. segment the edge for collision checking
    int steps = round(distance(state_from, state_to) / _mapResolution);
    float stepX = (state_to->x - state_from->x) / (float)steps;
    float stepY = (state_to->y - state_from->y) / (float)steps;
    float stepZ = (state_to->z - state_from->z) / (float)steps;
    // 2. allocate memory for a state, this state must be deleted after collision checking
    state_t *stateCur = new state_t;;
    stateCur->x = state_from->x;
    stateCur->y = state_from->y;
    stateCur->z = state_from->z;
    // 3. collision checking loop
    for (int stepCount = 0; stepCount < steps; ++stepCount) {
        stateCur->x += stepX;
        stateCur->y += stepY;
        stateCur->z += stepZ;
        if (isIncollision(stateCur->x, stateCur->y)) {
            delete stateCur;
            return false;
        }
        // close to obstacle cost, c1 cost.
        edgeCosts[0] += _mapResolution * getCloseCollisionCost(stateCur->x, stateCur->y);
    }
    delete stateCur;

    // lane change (heading) cost, c2 cost.
    edgeCosts[1] = (float)abs(state_from->idx - state_to->idx);

    // distance cost
    edgeCosts[2] = distance(state_from, state_to);

    // treat first column nodes all the same
    if (state_from->idy == 0 && state_to->idy == 0) {
        for (int i = 0; i < NUM_COSTS; ++i) {
            edgeCosts[i] = 0;
        }
    }
    // treat last column nodes all the same
    if (state_from->idy == adjacency_length_grid - 1 &&
        state_to->idy == adjacency_length_grid - 1) {
        for (int i = 0; i < NUM_COSTS; ++i) {
            edgeCosts[i] = 0;
        }
    }

    return true;
}

state_t* pathPlanning::minCostStateInQueue(std::vector<state_t*> Queue) {

    // Loop through cost hierarchy
    for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex) {
        std::vector<state_t*> tempQueue;
        float minCost = FLT_MAX;
        // loop through nodes saved in Queue
        for (std::vector<state_t*>::const_iterator iter2 = Queue.begin(); iter2 != Queue.end(); ++iter2) {
            state_t* thisState = *iter2;
            // if cost is lower, we put it in tempQueue in case other nodes can offer same cost
            if (thisState->costsToRoot[costIndex] < minCost) {
                minCost = thisState->costsToRoot[costIndex];
                tempQueue.clear();
                tempQueue.push_back(thisState);
            }
            // same cost can be offered by other nodes, we save them to tempQueue for next loop (secondary cost)
            else if (thisState->costsToRoot[costIndex] == minCost) {
                tempQueue.push_back(thisState);
            }
        }
        // Queue is used again for next loop
        Queue.clear();
        Queue = tempQueue;
    }
    // If cost hierarchy is selected correctly, there will be only one element left in Queue (no other ties)
    return Queue[0];
}

void pathPlanning::updateCosts(state_t* fromState, state_t* toState, int neighborInd) {

    for (int i = 0; i < NUM_COSTS; ++i) {
        toState->costsToRoot[i] = fromState->costsToRoot[i] +
            fromState->neighborList[neighborInd].edgeCosts[i];
    }
}

nav_msgs::Path pathPlanning::extractPath(state_t* stateCur) {

    nav_msgs::Path pathOut;
    pathOut.header.frame_id = "map";
    pathOut.header.stamp = ros::Time();
    while (ros::ok()) {
        geometry_msgs::PoseStamped poseCur;
        poseCur.header.stamp = ros::Time();
        poseCur.header.frame_id = "map";
        poseCur.pose.position.x = stateCur->x;
        poseCur.pose.position.y = stateCur->y;
        poseCur.pose.position.z = stateCur->z;
        pathOut.poses.insert(pathOut.poses.begin(), poseCur);
        if (stateCur->parentState == NULL) {
            break;
        }
        else {
            stateCur = stateCur->parentState;
        }
    }
    // 计算每个节点的朝向，补充 pathOut 的密度并平滑
    pathOut = processPath(pathOut);
    return pathOut;
}

void pathPlanning::buildAdjacencyMatrix() {

    // 返回 alternativePaths（rollouts）
    std::vector<nav_msgs::Path> alternativePaths;
    openPlannerRollOut.run(pathSmooth, alternativePaths);

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
    std::cout << "building adjacency matrix: " << adjacency_width_grid
        << " x " << adjacency_length_grid << std::endl;

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

void pathPlanning::connectAdjacencyMatrix() {

    // check node collision first
    for (int i = 0; i < nodeList.size(); ++i) {
        if (isIncollision(nodeList[i]->x, nodeList[i]->y)) {
            nodeList[i]->validFlag = false;
            continue;
        }
    }

    // connect adjacency matrix
    int connection_length = 1;
    for (int i = 0; i < adjacency_width_grid; ++i) {  // 16 行
        for (int j = 0; j < adjacency_length_grid; ++j) {
            if (adjacencyMatrix[i][j]->validFlag == false) {
                continue;
            }
            state_t* state_from = adjacencyMatrix[i][j];
            
            // 共有 8 个生长方向
            for (int m = -connection_length; m <= connection_length; ++m) {
                for (int n = -connection_length; n <= connection_length; ++n) {
                    // do not add itself
                    if (m == 0 && n == 0) {
                        continue;
                    }
                    int id_x = i + m;
                    int id_y = j + n;
                    if (id_x < 0 || id_x >= adjacency_width_grid ||
                        id_y < 0 || id_y >= adjacency_length_grid) {
                        continue;
                    }
                    // do not add collision node
                    if (adjacencyMatrix[id_x][id_y]->validFlag == false) {
                        continue;
                    }
                    state_t* state_to = adjacencyMatrix[id_x][id_y];

                    float edgeCosts[NUM_COSTS];  // 4
                    // 计算选择当前 edge 的 costs
                    if(edgePropagation(state_from, state_to, edgeCosts) == true) {
                        neighbor_t thisNeighbor;
                        for (int q = 0; q < NUM_COSTS; ++q) {
                            thisNeighbor.edgeCosts[q] = edgeCosts[q];
                        }
                        thisNeighbor.neighbor = state_to;
                        // neighborList 只维护可前往的邻居
                        state_from->neighborList.push_back(thisNeighbor);
                    }
                }
            }
        }
    } 
}

bool pathPlanning::searchAdjacencyMatrix() {

    // 1. reset costs
    for (int i = 0; i < nodeList.size(); ++i) {
        for (int j = 0; j < NUM_COSTS; ++j) {
            nodeList[i]->costsToRoot[j] = FLT_MAX;  // maximum value of a float.
        }
    }

    // 2. define start state
    state_t *startState = adjacencyMatrix[int(adjacency_width_grid/2)][0];
    for (int i = 0; i < NUM_COSTS; ++i) {
        startState->costsToRoot[i] = 0;
    }

    // 3. search graph
    // 穷举，记录各个节点的 min costsToRoot
    std::vector<state_t*> Queue;
    Queue.push_back(startState);
    while(Queue.size() > 0 && ros::ok()) {
        // find the state that can offer lowest cost in this depth and remove it from Queue
        state_t *fromState = minCostStateInQueue(Queue);
        Queue.erase(remove(Queue.begin(), Queue.end(), fromState), Queue.end());
        // loop through all neighbors of this state
        for (int i = 0; i < fromState->neighborList.size(); ++i) {
            state_t *toState = fromState->neighborList[i].neighbor;
            if (toState->validFlag == false) {
                continue;
            }
            // Loop through cost hierarchy
            for (int costIndex = 0; costIndex < NUM_COSTS; ++costIndex) {
                float thisCost = fromState->costsToRoot[costIndex] + fromState->neighborList[i].edgeCosts[costIndex];
                // If cost can be improved, update this node with new costs
                if (thisCost < toState->costsToRoot[costIndex]) {
                    updateCosts(fromState, toState, i);  // update toState's costToRoot
                    toState->parentState = fromState;    // change parent for toState
                    Queue.push_back(toState);
                }
                // If cost is same, go to compare secondary cost
                else if (thisCost == toState->costsToRoot[costIndex]) {
                    continue;
                }
                // If cost becomes higher, abort this propagation
                else {
                    break;
                }
            }
        }
    }

    // 4. find goal state
    // 查找最小的 costsToRoot
    Queue.clear();
    for (int i = 0; i < adjacency_width_grid; ++i) {  // <16
        Queue.push_back(adjacencyMatrix[i][adjacency_length_grid-1]);
    }
    state_t* goalState = minCostStateInQueue(Queue);

    // 5. extract path
    if (goalState->parentState == NULL) {
        std::cout << "No path found, stay stationary!" << std::endl;
    }
    else {
        // 循环查找父节点，连接为 searchedPath
        executePath = extractPath(goalState);
    }
    return true;
}

void pathPlanning::visualization() {

    if (pubPRMGraph.getNumSubscribers() != 0) {
        visualization_msgs::MarkerArray markerArray;
        geometry_msgs::Point p;

        // PRM nodes visualization
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = "map";
        markerNode.header.stamp = ros::Time::now();
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "nodes";
        markerNode.id = 2;
        markerNode.scale.x = 0.03; markerNode.scale.y = 0.03; markerNode.scale.z = 0.03; 
        markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
        markerNode.color.a = 1;
        for (int i = 0; i < adjacency_width_grid; ++i) {
            for (int j = 0; j < adjacency_length_grid; ++j) {
                if (adjacencyMatrix[i][j]->validFlag == false) {
                    continue;
                }
                p.x = adjacencyMatrix[i][j]->x;
                p.y = adjacencyMatrix[i][j]->y;
                p.z = adjacencyMatrix[i][j]->z + 0.015;
                markerNode.points.push_back(p);
            }
        }

        // PRM edge visualization
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = "map";
        markerEdge.header.stamp = ros::Time::now();
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "edges";
        markerEdge.id = 3;
        markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
        markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
        markerEdge.color.a = 1;
        for (int i = 0; i < adjacency_width_grid; ++i) {
            for (int j = 0; j < adjacency_length_grid; ++j) {
                if (adjacencyMatrix[i][j]->validFlag == false) {
                    continue;
                }
                int numNeighbors = adjacencyMatrix[i][j]->neighborList.size();
                for (int k = 0; k < numNeighbors; ++k) {
                    if (adjacencyMatrix[i][j]->neighborList[k].neighbor->validFlag == false) {
                        continue;
                    }
                    p.x = adjacencyMatrix[i][j]->x;
                    p.y = adjacencyMatrix[i][j]->y;
                    p.z = adjacencyMatrix[i][j]->z + 0.005;
                    markerEdge.points.push_back(p);
                    p.x = adjacencyMatrix[i][j]->neighborList[k].neighbor->x;
                    p.y = adjacencyMatrix[i][j]->neighborList[k].neighbor->y;
                    p.z = adjacencyMatrix[i][j]->neighborList[k].neighbor->z + 0.005;
                    markerEdge.points.push_back(p);
                }
            }
        }

        // push to markerarray and publish
        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubPRMGraph.publish(markerArray);
        // publish the path
        pubExecutePath.publish(executePath);
    }
}

/* 穷举，找到一条 cost 最小的局部路径*/
void pathPlanning::generatePath() {

    if (path_generated) {
        return;
    }

    if (path_published && pc_published) {
        // 建立邻接矩阵，16 行 n 列。储存每个节点的 x,y,theta
        buildAdjacencyMatrix();
        // 计算每个节点可去的 8 个邻接节点的 costs
        connectAdjacencyMatrix();
        // 搜索找到一条新的局部路径
        searchAdjacencyMatrix();
        // 可视化
        visualization();
        path_generated = true;
    }

}