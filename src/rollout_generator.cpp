/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-03-15 11:50:55
 */

#include "rollout_generator.h"

namespace UtilityNS {

/* 获取轨迹上距离当前位置最近的轨迹点（前方）*/
int getNextClosePointIndex(const vector<UtilityNS::WayPoint>& trajectory,
    const UtilityNS::WayPoint& curr_pos, const int& prevIndex) {

    if (trajectory.size() < 2 || prevIndex < 0) {
        return 0;
    }

    double dis = 0, min_dis = DBL_MAX;
    int min_index = prevIndex;  // =0

    for (int i = prevIndex; i < trajectory.size(); i++) {
        dis = distance2points_pow(trajectory[i].pos, curr_pos.pos);

        if (dis < min_dis) {
            min_index = i;
            min_dis = dis;
        }
    }

    // 有可能最近点在后方，此时 +1 就取到了前方的最近点
    if (min_index < (int)trajectory.size() - 2) {
        UtilityNS::GPSPoint closest, next;
        closest = trajectory[min_index].pos;
        next = trajectory[min_index + 1].pos;
        UtilityNS::GPSPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
        double length1 = calLength(v_1);
        UtilityNS::GPSPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
        double length2 = calLength(v_2);
        double angle = cast_from_0_to_2PI_Angle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
        if (angle <= M_PI_2)
            min_index = min_index + 1;
    }
    return min_index;
}

/**
 * @description: 显示一条车道线（测试） 
 * @param {type} 
 * @return: 
 */
void visualLaneInRviz(const vector<UtilityNS::WayPoint>& lane, ros::Publisher pub_testLane)
{
    if (pub_testLane.getNumSubscribers() == 0)
        return;
    visualization_msgs::Marker lane_marker;

    lane_marker.header.frame_id = "map";
    lane_marker.header.stamp = ros::Time();
    lane_marker.ns = "test_lane";
    lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_marker.action = visualization_msgs::Marker::ADD;
    lane_marker.frame_locked = false;

    lane_marker.scale.x = 0.02;
    lane_marker.frame_locked = false;

    lane_marker.points.clear();
    for (size_t k = 0; k < lane.size(); k++) {
        geometry_msgs::Point wp;
        wp.x = lane[k].pos.x;
        wp.y = lane[k].pos.y;
        wp.z = lane[k].pos.z;
        lane_marker.points.push_back(wp);
    }

    lane_marker.color.b = 0;
    lane_marker.color.g = 0;
    lane_marker.color.r = 1;
    lane_marker.color.a = 1;

    pub_testLane.publish(lane_marker);
}

/**
 * @description:将当前角度转换到0～2pi 
 * @param {type} 
 * @return: 
 */
double cast_from_0_to_2PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < 0) {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}


double diffBetweenTwoAngle(const double& a1, const double& a2) {
    double diff = a1 - a2;
    if (diff < 0)
        diff = -diff;
    if (diff > M_PI)
        diff = 2 * M_PI - diff;
    return diff;
}

/* 计算某一个轨迹到某一个点的相对位置*/
bool getRelativeInfo(const vector<UtilityNS::WayPoint>& trajectory,
    const UtilityNS::WayPoint& p, UtilityNS::RelativeInfo& info) {

    if (trajectory.size() < 2) {
        return false;
    }

    UtilityNS::WayPoint p0, p1;
    if (trajectory.size() == 2) {
        p0 = trajectory[0];
        p1 = UtilityNS::WayPoint((p0.pos.x + trajectory[1].pos.x) / 2.0,
            (p0.pos.y + trajectory[1].pos.y) / 2.0,
            (p0.pos.z + trajectory[1].pos.z) / 2.0,
            p0.pos.yaw);
        info.iBack = 0;
        info.iFront = 1;
    }
    else {
        info.iFront = getNextClosePointIndex(trajectory, p);  // 后一个点的 index
        if (info.iFront > 0) {
            info.iBack = info.iFront - 1;  // 前一个点的 index，有用
        }
        else{
            info.iBack = 0;
        }

        if (info.iFront == 0) {
            p0 = trajectory[info.iFront];
            p1 = trajectory[info.iFront + 1];
        }
        else if (info.iFront > 0 && info.iFront < trajectory.size() - 1) {
            p0 = trajectory[info.iFront - 1];  // 前一个点
            p1 = trajectory[info.iFront];      // 后一个点
        }
        else {
            p0 = trajectory[info.iFront - 1];
            p1 = UtilityNS::WayPoint((p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0,
                p0.pos.yaw);
        }
    }

    UtilityNS::WayPoint prevWP = p0;
    UtilityNS::Mat3 rotationMat(-p1.pos.yaw);            // 绕 z 轴旋转 double
    UtilityNS::Mat3 translationMat(-p.pos.x, -p.pos.y);  // x-y 平面平移
    UtilityNS::Mat3 invRotationMat(p1.pos.yaw);
    UtilityNS::Mat3 invTranslationMat(p.pos.x, p.pos.y);

    p0.pos = translationMat * p0.pos;
    p0.pos = rotationMat * p0.pos;
    p1.pos = translationMat * p1.pos;
    p1.pos = rotationMat * p1.pos;

    double k = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
    info.perp_distance = p1.pos.y - k * p1.pos.x;  // 估计的 p 到 trajectory 的距离，有用
    if (isnan(info.perp_distance) || isinf(info.perp_distance)) {
        info.perp_distance = 0;
    }

    info.to_front_distance = fabs(p1.pos.x);

    info.perp_point = p1;
    info.perp_point.pos.x = 0;
    info.perp_point.pos.y = info.perp_distance;
    info.perp_point.pos = invRotationMat * info.perp_point.pos;
    info.perp_point.pos = invTranslationMat * info.perp_point.pos;

    info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
    info.angle_diff = UtilityNS::diffBetweenTwoAngle(p1.pos.yaw, p.pos.yaw) * RAD2DEG_ME;

    info.direct_distance = hypot(p1.pos.y - p.pos.y, p1.pos.x - p.pos.x);
    return true;
}

} // namespace UtilityNS



namespace RolloutGeneratorNS {

RolloutGenerator::RolloutGenerator()
    : nh_private("") {

    initROS();
    currentPose_flag = false;
}

RolloutGenerator::~RolloutGenerator() {}

void RolloutGenerator::initROS() {
    nh_private.param<double>("roboat_planning/_samplingTipMargin", PlanningParams.carTipMargin, 0.5);
    nh_private.param<double>("roboat_planning/_samplingOutMargin", PlanningParams.rollInMargin, 0.2);
    nh_private.param<double>("roboat_planning/_samplingSpeedFactor", PlanningParams.rollInSpeedFactor, 0.25);

    nh_private.param<double>("roboat_planning/_pathResolution", PlanningParams.pathDensity, 0.1);
    nh_private.param<double>("roboat_planning/_maxPathDistance", PlanningParams.microPlanDistance, 10.0);

    nh_private.param<double>("roboat_planning/_rollOutDensity", PlanningParams.rollOutDensity, 0.1);
    nh_private.param<int>("roboat_planning/_rollOutNumber", PlanningParams.rollOutNumber, 15);

    nh_private.param<double>("roboat_planning/_smoothingDataWeight", PlanningParams.smoothingDataWeight, 0.45);
    nh_private.param<double>("roboat_planning/_smoothingSmoothWeight", PlanningParams.smoothingSmoothWeight, 0.4);
    nh_private.param<double>("roboat_planning/_smoothingToleranceError", PlanningParams.smoothingToleranceError, 0.05);

    nh_private.param<double>("roboat_planning/_speedProfileFactor", PlanningParams.speedProfileFactor, 1.2);

    pub_localTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("planning/op/rollouts_all", 1);
    pub_testLane = nh.advertise<visualization_msgs::Marker>("planning/op/rollouts_center", 1);

    pub_global_path = nh.advertise<nav_msgs::Path>("planning/op/global_path", 1);
    pub_center_path = nh.advertise<nav_msgs::Path>("planning/op/center_path", 1);
    pub_remaining_path = nh.advertise<nav_msgs::Path>("planning/op/remaining_path", 1);

    speed = 1;
}

/* 主循环函数*/
void RolloutGenerator::run(nav_msgs::Path pathMsg, vector<nav_msgs::Path>& alternativePaths) {

    // 把 trajectory 信息（包括每个路径点的 x,y,z,yaw,cost）写入 globalPaths
    getGlobalPlannerPath_cb(pathMsg);
    if (globalPaths.size() == 0) {
        return;
    }

    // 从 globalPaths[i] 拷贝到局部路径 centralTrajectorySmoothed，对局部路径插值并计算 angle and cost。
    globalPathSections.clear();
    for (size_t i = 0; i < globalPaths.size(); i++) {
        centralTrajectorySmoothed.clear();
        extractPartFromTrajectory(globalPaths[i],
            PlanningParams.pathDensity, centralTrajectorySmoothed);  // 10.0, 0.1,
        globalPathSections.push_back(centralTrajectorySmoothed);
    }

    // 由 globalPathSections 生成 16 条先扇形展开后扇形收敛的 rollouts，并平滑*/
    generateRunoffTrajectory(globalPathSections,
        speed,                                   // 1
        PlanningParams.carTipMargin,             // 0.5
        PlanningParams.rollInMargin,             // 0.2
        PlanningParams.speedProfileFactor,       // 1.2
        PlanningParams.pathDensity,              // 0.1
        PlanningParams.rollOutDensity,           // 0.1
        PlanningParams.rollOutNumber,            // 15
        PlanningParams.smoothingDataWeight,      // 0.45
        PlanningParams.smoothingSmoothWeight,    // 0.4
        PlanningParams.smoothingToleranceError,  // 0.05
        rollOuts);

    // 将 16 条 rollouts 转化为 vector<nav_msgs::Path> 数据格式
    alternativePaths = getOtherpaths(rollOuts[0]);

    // re-calculate orientation
    // fixedGlobal = calculatePathYaw(fixedGlobal);
    // centerPath = calculatePathYaw(centerPath);
    // remainingPath = calculatePathYaw(remainingPath);
    // for (int i = 0; i < alternativePaths.size(); ++i)
    //     alternativePaths[i] = calculatePathYaw(alternativePaths[i]);
}

nav_msgs::Path RolloutGenerator::calculatePathYaw(nav_msgs::Path pathIn)
{
    int length = pathIn.poses.size();
    if (length <= 1)
    {
        if (length == 1)
            pathIn.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        return pathIn;
    }

    for (int i = 0; i < length - 1; ++i)
    {
        double dx = pathIn.poses[i+1].pose.position.x - pathIn.poses[i].pose.position.x;
        double dy = pathIn.poses[i+1].pose.position.y - pathIn.poses[i].pose.position.y;
        double theta = atan2(dy, dx);
        pathIn.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }

    pathIn.poses.back().pose.orientation = pathIn.poses[length-2].pose.orientation;

    return pathIn;
}

/* 截取 pathIn 中 centerPath 之后的部分*/
nav_msgs::Path RolloutGenerator::getRemainingPath(nav_msgs::Path pathIn, nav_msgs::Path centerPath) {

    nav_msgs::Path pathOut = pathIn;
    geometry_msgs::Point lastPose = centerPath.poses.back().pose.position;
    for (int i = 0; i < pathIn.poses.size(); ++i) {
        geometry_msgs::Point curPose = pathIn.poses[i].pose.position;
        if (sqrt((lastPose.x-curPose.x)*(lastPose.x-curPose.x) + 
            (lastPose.y-curPose.y)*(lastPose.y-curPose.y)) <= PlanningParams.pathDensity) {

            if (i + 1 <= pathOut.poses.size() -1) {
                pathOut.poses.erase(pathOut.poses.begin(), pathOut.poses.begin() + i + 1);
            }
            else {
                pathOut.poses.erase(pathOut.poses.begin(), pathOut.poses.begin() + i);
            }
            break;
        }
    }

    return pathOut;
}

/* 将 pathIn 转化为 nav_msgs::Path 格式*/
nav_msgs::Path RolloutGenerator::getCenterPath(vector<UtilityNS::WayPoint> pathIn) {

    nav_msgs::Path pathOut;
    pathOut.header.stamp = ros::Time();
    pathOut.header.frame_id = "map";

    for (int i = 0; i < pathIn.size(); ++i) {
        geometry_msgs::PoseStamped poseCur;
        poseCur.header.stamp = ros::Time();
        poseCur.header.frame_id = "map";

        poseCur.pose.position.x = pathIn[i].pos.x;
        poseCur.pose.position.y = pathIn[i].pos.y;
        poseCur.pose.position.z = pathIn[i].pos.z;

        poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(pathIn[i].pos.yaw);

        pathOut.poses.push_back(poseCur);
    }

    return pathOut;
}

/* 将 pathsIn 转化为 vector<nav_msgs::Path> 数据格式*/
vector<nav_msgs::Path> RolloutGenerator::getOtherpaths(vector<vector<UtilityNS::WayPoint>> pathsIn) {

    vector<nav_msgs::Path> pathVecOut;
    nav_msgs::Path pathOut;
    pathOut.header.stamp = ros::Time();
    pathOut.header.frame_id = "map";

    for (int i = 0; i < pathsIn.size(); ++i) {  // 16
        pathOut.poses.clear();
        for (int j = 0; j < pathsIn[i].size(); ++j) {
            geometry_msgs::PoseStamped poseCur;
            poseCur.header.stamp = ros::Time();
            poseCur.header.frame_id = "map";
            poseCur.pose.position.x = pathsIn[i][j].pos.x;
            poseCur.pose.position.y = pathsIn[i][j].pos.y;
            poseCur.pose.position.z = pathsIn[i][j].pos.z;
            poseCur.pose.orientation = tf::createQuaternionMsgFromYaw(pathsIn[i][j].pos.yaw);
            pathOut.poses.push_back(poseCur);
        }
        pathVecOut.push_back(pathOut);
    }

    return pathVecOut;
}

void RolloutGenerator::trajectoryToMarkers(const vector<vector<vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "rollouts";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.01;
    lane_waypoint_marker.frame_locked = false;
    lane_waypoint_marker.color.r = 0.0; lane_waypoint_marker.color.g = 1.0; lane_waypoint_marker.color.b = 0.0;
    lane_waypoint_marker.color.a = 0.7;

    for (size_t i = 0; i < paths.size(); i++) {
        for (size_t k = 0; k < paths[i].size(); k++) {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = i * 10 + k;
            for (size_t m = 0; m < paths[i][k].size(); m++) {
                geometry_msgs::Point wp;
                wp.x = paths[i][k][m].pos.x;
                wp.y = paths[i][k][m].pos.y;
                wp.z = paths[i][k][m].pos.z;
                lane_waypoint_marker.points.push_back(wp);
            }
            markerArray.markers.push_back(lane_waypoint_marker);
        }
    }
    // nodes visualization
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "map";
    markerNode.header.stamp = ros::Time();
    markerNode.ns = "nodes";
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.id = 999999;
    markerNode.scale.x = 0.02;
    markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
    markerNode.color.a = 0.7;

    for (size_t i = 0; i < paths.size(); i++) {
        for (size_t k = 0; k < paths[i].size(); k++) {
            for (size_t m = 0; m < paths[i][k].size(); m++) {
                geometry_msgs::Point wp;
                wp.x = paths[i][k][m].pos.x;
                wp.y = paths[i][k][m].pos.y;
                wp.z = paths[i][k][m].pos.z;
                markerNode.points.push_back(wp);
            }
        }
    }
    markerArray.markers.push_back(markerNode);
}

/* 生成候选局部规划路径 rollouts*/
void RolloutGenerator::generateRunoffTrajectory(const vector<vector<UtilityNS::WayPoint>>& referencePaths,
    const double& speed,              // 1
    const double& carTipMargin,       // 0.5
    const double& rollInMargin,       // 0.2
    const double& rollInSpeedFactor,  // 1.2
    const double& pathDensity,        // 0.1
    const double& rollOutDensity,     // 0.1
    const int& rollOutNumber,         // 15
    const double& SmoothDataWeight,   // 0.45
    const double& SmoothWeight,       // 0.4
    const double& SmoothTolerance,    // 0.05
    vector<vector<vector<UtilityNS::WayPoint>>>& rollOutsPaths) {  // 每条 referencePaths 包含多条 rollOutsPaths

    if ((referencePaths.size() == 0)) {
        return;
    }

    rollOutsPaths.clear();
    for (unsigned int i = 0; i < referencePaths.size(); i++) {
        vector<vector<UtilityNS::WayPoint>> local_rollOutPaths;
        if (referencePaths.at(i).size() > 0) {
            calculateRollInTrajectories(speed, referencePaths.at(i),
                local_rollOutPaths, carTipMargin, rollInMargin,
                rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
                SmoothDataWeight, SmoothWeight, SmoothTolerance);
        }
        rollOutsPaths.push_back(local_rollOutPaths);
    }
}

/* 由中心轨迹的采样点生成 16 条先扇形展开后扇形收敛的 rollouts，并平滑*/
void RolloutGenerator::calculateRollInTrajectories(
    const double& speed,  // 1
    const vector<UtilityNS::WayPoint>& originalCenter,  // 截取出来的局部路径
    vector<vector<UtilityNS::WayPoint>>& rollInPaths,   // 输出 rollouts
    const double& carTipMargin,       // 0.5
    const double& rollInMargin,       // 0.2
    const double& rollInSpeedFactor,  // 1.2
    const double& pathDensity,        // 0.1
    const double& rollOutDensity,     // 0.1
    const int& rollOutNumber,         // 15
    const double& SmoothDataWeight,   // 0.45
    const double& SmoothWeight,       // 0.4
    const double& SmoothTolerance) {  // 0.05

    double remaining_distance = 0;  // trajectory 上剩余路程的长度
    for (unsigned int i = 0; i < originalCenter.size() - 1; i++) {
        remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    // 计算 start_distance：第一段的长度
    double start_distance = rollInSpeedFactor * speed + rollInMargin;  // 1.2*1+0.2
    if (start_distance > remaining_distance) {
        start_distance = remaining_distance;
    }

    // 计算 far_index：第一段结束的 index
    double d_limit = 0;
    unsigned int far_index = 0;
    for (unsigned int i = 1; i < originalCenter.size(); i++) {
        d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit >= start_distance) {
            far_index = i;
            break;
        }
    }

    // 计算 16 条 rollouts 路径到中心路径的距离
    int centralTrajectoryIndex = rollOutNumber / 2;  // 15/2 = 7
    vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++) {  // 长 16
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);  // 0.1*(i-7)
        end_distance_list.push_back(end_roll_in_distance);
    }

    // 计算 smoothing_start_index：第一段开始的 index
    d_limit = 0;
    unsigned int smoothing_start_index = 0;
    for (unsigned int i = 1; i < originalCenter.size(); i++) {
        d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin) {  // >0.5
            smoothing_start_index = i;
            break;
        }
    }
    // 计算 smoothing_end_index：第一段结束的 index
    d_limit = 0;
    unsigned int smoothing_end_index = far_index;
    for (unsigned int i = far_index; i < originalCenter.size(); i++) {
        d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin) {  // >0.5
            smoothing_end_index = i;
            break;
        }
    }
    
    // 一些初始化
    double initial_roll_in_distance = 0;  // car 到 trajectory 的距离
    int nSteps = far_index - smoothing_start_index;
    rollInPaths.clear();
    vector<double> inc_list;
    vector<double> inc_list_inc;
    for (int i = 0; i < rollOutNumber + 1; i++) {
        rollInPaths.push_back(vector<UtilityNS::WayPoint>());
        double diff = end_distance_list.at(i) - initial_roll_in_distance;  // 0.1*(i-7)-irid
        inc_list.push_back(diff / (double)nSteps);  // 扇形，距离越远，越扩散
        inc_list_inc.push_back(0);
    }

    // 第一段中，从 0 到 smoothing_start_index 这一部分
    // 先平行于 trajectory 运动一小段路程
    UtilityNS::WayPoint p;
    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;  // 16.666
    if (iLimitIndex >= originalCenter.size()) {
        iLimitIndex = originalCenter.size() - 1;
    }
    for (unsigned int j = 0; j < smoothing_start_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;  // 1
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2);
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;  // 1*0.5
            }
            else {
                p.v = original_speed;  // 1
            }

            rollInPaths.at(i).push_back(p);  // 最终输出的
        }
    }

    // 第一段中，从 smoothing_start_index 到 far_index 这一部分
    // 开始扇形展开
    for (unsigned int j = smoothing_start_index; j < far_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;  // 1
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] += inc_list[i];  // 随着 j 增大，等差增大
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.yaw + M_PI_2) - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.yaw + M_PI_2) - d * sin(p.pos.yaw + M_PI_2);
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;  // 1*0.5
            }
            else {
                p.v = original_speed;  // 1
            }

            rollInPaths.at(i).push_back(p);  // 最终输出的
        }
    }
    
    // 第一段中，从 far_index 到 smoothing_end_index 这一部分
    // 扇形展开完，并入 trajectory
    for (unsigned int j = far_index; j < smoothing_end_index; j++) {
        p = originalCenter.at(j);
        double original_speed = p.v;  // 1
        for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
            double d = end_distance_list.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);  // 并入 trajectory，不用 irid
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;  // 1*0.5
            }
            else {
                p.v = original_speed;  // 1
            }

            rollInPaths.at(i).push_back(p);  // 最终输出的
        }
    }

    // 第二段，平行于 trajectory
    d_limit = 0; 
    for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++) {
        d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);
        p = originalCenter.at(j);
        double original_speed = p.v;  // 1
        for (unsigned int i = 0; i < rollInPaths.size(); i++) {
            double d = end_distance_list.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.yaw + M_PI_2);
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;  // 1*0.5
            }
            else {
                p.v = original_speed;  // 1
            }

            rollInPaths.at(i).push_back(p);  // 最终输出的
        }
    }

    // 将第二段部分修正为第三段的末尾部分
    // 功能等同于第一段的从 0 到 smoothing_start_index 这一部分
    int path_length = rollInPaths[0].size();
    int seg_num_2 = smoothing_start_index;  // 第一段中的第一部分
    int j_start = path_length - seg_num_2;  // 剩余部分总和
    for (int j = j_start; j < path_length; j++) {
        int origin_path_index = min(int(j), (int)originalCenter.size() - 1);
        p = originalCenter.at(origin_path_index);
        double original_speed = p.v;
        for (int i = 0; i < rollOutNumber + 1; i++) {
            p.pos.x = originalCenter.at(origin_path_index).pos.x;  // 收敛回 trajectory
            p.pos.y = originalCenter.at(origin_path_index).pos.y;
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            }
            else {
                p.v = original_speed;
            }

            rollInPaths[i][j] = p;  // 最终输出的
        }
    }

    // 修正扇形
    inc_list.clear();
    inc_list_inc.clear();
    for (int i = 0; i < rollOutNumber + 1; i++) {
        double diff = end_distance_list.at(i);      // 0.1*(i-8)
        inc_list.push_back(diff / (double)nSteps);  // 扇形，距离越远，越扩散
        inc_list_inc.push_back(0);
    }
    for (int j = smoothing_start_index; j < far_index; j++) {
        for (int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] += inc_list[i];
        }
    }
    // 将第二段部分修正为第三段的倒数第二部分
    // 功能等同于第一段的从 smoothing_start_index 到 far_index 这一部分
    int seg_num_1 = far_index - smoothing_start_index;  // 第一段中的第二部分（扇形展开）
    j_start = max(path_length - seg_num_2 - seg_num_1, int(smoothing_start_index));
    for (int j = j_start; j < path_length - seg_num_2; j++) {
        int origin_path_index = min(int(j), (int)originalCenter.size() - 1);
        p = originalCenter.at(origin_path_index);
        double original_speed = p.v;
        for (int i = 0; i < rollOutNumber + 1; i++) {
            inc_list_inc[i] -= inc_list[i];
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(origin_path_index).pos.x - d * cos(p.pos.yaw + M_PI_2);
            p.pos.y = originalCenter.at(origin_path_index).pos.y - d * sin(p.pos.yaw + M_PI_2);
            if (i != centralTrajectoryIndex) {
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            }
            else {
                p.v = original_speed;
            }

            rollInPaths[i][j] = p;  // 最终输出的
        }
    }


    // smooth path
    for (unsigned int i = 0; i < rollOutNumber + 1; i++) {
        smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }
}

/* 平滑生成的曲线*/
void RolloutGenerator::smoothPath(vector<UtilityNS::WayPoint>& path, double weight_data,
    double weight_smooth, double tolerance) {

    if (path.size() <= 2) {
        return;
    }

    const vector<UtilityNS::WayPoint>& path_in = path;
    vector<UtilityNS::WayPoint> smoothPath_out = path_in;
    double change = tolerance;  // 0.05
    double xtemp, ytemp;
    int nIterations = 0;
    int size = path_in.size();
    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < size - 1; i++) {
            xtemp = smoothPath_out[i].pos.x;
            ytemp = smoothPath_out[i].pos.y;

            smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
            smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

            smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
            smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

            change += fabs(xtemp - smoothPath_out[i].pos.x);
            change += fabs(ytemp - smoothPath_out[i].pos.y);
        }
        nIterations++;
    }
    path = smoothPath_out;
}

/* 从全局路径截取局部路径，对局部路径插值并计算 angle and cost。*/
void RolloutGenerator::extractPartFromTrajectory(const vector<UtilityNS::WayPoint>& originalPath,
    const double& waypointDensity, vector<UtilityNS::WayPoint>& extractedPath) {

    if (originalPath.size() < 2) {
        return;
    }

    extractedPath.clear();
    for (int i = 0; i < (int)originalPath.size(); i++) {
        extractedPath.push_back(originalPath[i]);
    }

    // 根据 waypointDensity 对 extractedPath 插值
    fixPathDensity(extractedPath, waypointDensity);  // 0.1
    // 利用前后点位置计算每个路径点的 yaw (0~2pi)
    // 把经过的路程作为每个路径点的 cost（单调递增）
    calcAngleAndCost(extractedPath);
}

/* 根据 pathDensity 对 path 插值*/
void RolloutGenerator::fixPathDensity(vector<UtilityNS::WayPoint>& path, const double& pathDensity) {

    if (path.size() == 0 || pathDensity == 0) {
        return;
    }

    double dis = 0, ang = 0;
    double margin = pathDensity * 0.01;  // 0.1*0.01
    double remaining = 0;
    int nPoints = 0;
    std::vector<UtilityNS::WayPoint> fixedPath;
    fixedPath.push_back(path[0]);
    size_t start = 0, next = 1;
    while (next < path.size()) {
        dis += hypot(path[next].pos.x - path[next - 1].pos.x,
            path[next].pos.y - path[next - 1].pos.y) + remaining;
        ang = atan2(path[next].pos.y - path[start].pos.y, path[next].pos.x - path[start].pos.x);

        if (dis < pathDensity - margin) {
            next++;
            remaining = 0;
        }
        // 需要插值
        else if (dis > (pathDensity + margin)) {
            UtilityNS::WayPoint point_start = path[start];
            nPoints = dis / pathDensity;
            for (int j = 0; j < nPoints; j++) {
                point_start.pos.x += pathDensity * cos(ang);
                point_start.pos.y += pathDensity * sin(ang);
                fixedPath.push_back(point_start);
            }
            remaining = dis - nPoints * pathDensity;
            start++;
            path[start].pos = point_start.pos;
            dis = 0;
            next++;
        }
        else {
            dis = 0;
            remaining = 0;
            fixedPath.push_back(path[next]);
            next++;
            start = next - 1;
        }
    }
    path = fixedPath;
}

/* 根据 PlanningParams.pathDensity 对 path 插值*/
nav_msgs::Path RolloutGenerator::fixPathDensity(nav_msgs::Path path) {

    double _pathResolution = PlanningParams.pathDensity;
    if (path.poses.size() == 0 || _pathResolution == 0) {
        return path;
    }

    double dis = 0, ang = 0;
    double margin = _pathResolution * 0.01;
    double remaining = 0;
    int nPoints = 0;
    nav_msgs::Path fixedPath = path;
    fixedPath.poses.clear();
    fixedPath.poses.push_back(path.poses[0]);
    size_t start = 0, next = 1;
    while (next < path.poses.size()) {
        dis += hypot(path.poses[next].pose.position.x - path.poses[next-1].pose.position.x,
            path.poses[next].pose.position.y - path.poses[next-1].pose.position.y) + remaining;
        ang = atan2(path.poses[next].pose.position.y - path.poses[start].pose.position.y,
            path.poses[next].pose.position.x - path.poses[start].pose.position.x);

        if (dis < _pathResolution - margin) {
            next++;
            remaining = 0;
        }
        // 需要插值
        else if (dis > (_pathResolution + margin)) {
            geometry_msgs::PoseStamped point_start = path.poses[start];
            nPoints = dis / _pathResolution;
            for (int j = 0; j < nPoints; j++) {
                point_start.pose.position.x = point_start.pose.position.x + _pathResolution * cos(ang);
                point_start.pose.position.y = point_start.pose.position.y + _pathResolution * sin(ang);
                point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
                fixedPath.poses.push_back(point_start);
            }
            remaining = dis - nPoints * _pathResolution;
            start++;
            path.poses[start].pose.position = point_start.pose.position;
            dis = 0;
            next++;
        }
        else {
            dis = 0;
            remaining = 0;
            fixedPath.poses.push_back(path.poses[next]);
            next++;
            start = next - 1;
        }
    }
    return fixedPath;
}

/* 把信息（包括每个路径点的 x,y,z,yaw,cost）写入 globalPaths*/
void RolloutGenerator::getGlobalPlannerPath_cb(nav_msgs::Path msg) {
    if (msg.poses.size() > 0) {
        globalPaths.clear();
        std::vector<UtilityNS::WayPoint> single_path;
        // 将 nav_msgs::Path 数据格式存为 std::vector<UtilityNS::WayPoint>
        msgLane2LocalLane(msg, single_path);
        // 利用前后点位置计算每个路径点的 yaw (0~2pi)
        // 把经过的路程作为每个路径点的 cost（单调递增）
        calcAngleAndCost(single_path);
        globalPaths.push_back(single_path);
    }
}

/* 将 nav_msgs::Path 数据格式存为 std::vector<UtilityNS::WayPoint>*/
void RolloutGenerator::msgLane2LocalLane(nav_msgs::Path msg_path, vector<UtilityNS::WayPoint>& path) {
    path.clear();
    for (size_t i = 0; i < msg_path.poses.size(); i++) {
        UtilityNS::WayPoint wp;
        wp.pos.x = msg_path.poses[i].pose.position.x;
        wp.pos.y = msg_path.poses[i].pose.position.y;
        wp.pos.z = msg_path.poses[i].pose.position.z;
        wp.pos.yaw = tf::getYaw(msg_path.poses[i].pose.orientation);
        wp.v = 1;  //msg_path.waypoints.at(i).speed_limit;
        // wp.laneId = msg_path.waypoints.at(i).lane_id;
        path.push_back(wp);
    }
}

/* 利用前后点位置计算每个路径点的 yaw (0~2pi)；把经过的路程作为每个路径点的 cost（单调递增）*/
double RolloutGenerator::calcAngleAndCost(vector<UtilityNS::WayPoint>& path) {

    if (path.size() < 2) {
        return 0;
    }

    if (path.size() == 2) {
        path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = 0;
        path[1].pos.yaw = path[0].pos.yaw;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = 0;
    for (int j = 1; j < path.size() - 1; j++) {
        path[j].pos.yaw = UtilityNS::cast_from_0_to_2PI_Angle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }
    int j = (int)path.size() - 1;
    path[j].pos.yaw = path[j - 1].pos.yaw;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    return path[j].cost;
}

} // namespace RolloutGeneratorNS