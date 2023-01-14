/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @Date: 2019-02-04 11:13:45
 * @LastEditors: sunm
 * @LastEditTime: 2019-03-15 11:50:55
 */

#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

using namespace std;

namespace UtilityNS {

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
#define RAD2DEG_ME 180. / M_PI

class GPSPoint {
public:
    double x;
    double y;
    double z;
    double yaw;

    GPSPoint()
    {
        x = 0;
        y = 0;
        z = 0;
        yaw = 0;
    }

    GPSPoint(const double& x, const double& y, const double& z, const double& yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->yaw = yaw;
    }

    string ToString()
    {
        stringstream str;
        str.precision(12);
        str << "X:" << x << ", Y:" << y << ", Z:" << z << ", Yaw:" << yaw << endl;
        return str.str();
    }
};

class Rotation {
public:
    double x;
    double y;
    double z;
    double w;

    Rotation()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

class WayPoint {
public:
    GPSPoint pos;
    Rotation rot;
    double v;
    double cost;
    int laneId;

    WayPoint()
    {
        v = 0;
        cost = 0;
        laneId = -1;
    }

    WayPoint(const double& x, const double& y, const double& z, const double& a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.yaw = a;

        v = 0;
        cost = 0;
        laneId = -1;
    }
};

class Mat3 {
    double m[3][3];

public:
    Mat3()
    {
        //initialize Identity by default
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = 0;

        m[0][0] = m[1][1] = m[2][2] = 1;
    }

    Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
    {
        m[0][0] = (mirrorX == true) ? -1 : 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = (mirrorY == true) ? -1 : 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double transX, double transY)
    {
        m[0][0] = 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double rotation_angle)
    {
        double c = cos(rotation_angle);
        double s = sin(rotation_angle);
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = 0;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = 0;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(GPSPoint rotationCenter)
    {
        double c = cos(rotationCenter.yaw);
        double s = sin(rotationCenter.yaw);
        double u = rotationCenter.x;
        double v = rotationCenter.y;
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = -u * c + v * s + u;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = -u * s - v * c + v;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    GPSPoint operator*(GPSPoint v) {
        GPSPoint _v = v;
        v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
        v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
        return v;
    }
};

class RelativeInfo {
public:
    double perp_distance;
    double to_front_distance; //negative
    double from_back_distance;
    int iFront;
    int iBack;
    WayPoint perp_point;
    double angle_diff; // degrees
    double direct_distance;

    RelativeInfo()
    {
        perp_distance = 0;
        to_front_distance = 0;
        iFront = 0;
        iBack = 0;
        angle_diff = 0;
        direct_distance = 0;
    }
};

class PlanningParams {
public:
    double maxSpeed;
    double minSpeed;
    double planningDistance;
    double microPlanDistance;
    double carTipMargin;
    double rollInMargin;
    double rollInSpeedFactor;
    double pathDensity;
    double rollOutDensity;
    int rollOutNumber;
    double horizonDistance;
    double minFollowingDistance; //should be bigger than Distance to follow
    double minDistanceToAvoid;   // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
    double maxDistanceToAvoid;   // should be smaller than minDistanceToAvoid
    double lateralSkipDistance;
    double speedProfileFactor;
    double smoothingDataWeight;
    double smoothingSmoothWeight;
    double smoothingToleranceError;

    double stopSignStopTime;

    double additionalBrakingDistance;
    double verticalSafetyDistance;
    double horizontalSafetyDistancel;

    double giveUpDistance;

    int nReliableCount;

    bool enableLaneChange;
    bool enableSwerving;
    bool enableFollowing;
    bool enableHeadingSmoothing;
    bool enableTrafficLightBehavior;
    bool enableStopSignBehavior;

    bool enabTrajectoryVelocities;
    double minIndicationDistance;

    PlanningParams()
    {
        maxSpeed = 3;
        lateralSkipDistance = 6.0;
        minSpeed = 0;
        planningDistance = 10000;
        microPlanDistance = 30;
        carTipMargin = 4.0;
        rollInMargin = 12.0;
        rollInSpeedFactor = 0.25;
        pathDensity = 0.25;
        rollOutDensity = 0.5;
        rollOutNumber = 4;
        horizonDistance = 120;
        minFollowingDistance = 35;
        minDistanceToAvoid = 15;
        maxDistanceToAvoid = 5;
        speedProfileFactor = 1.0;
        smoothingDataWeight = 0.47;
        smoothingSmoothWeight = 0.2;
        smoothingToleranceError = 0.05;

        stopSignStopTime = 2.0;

        additionalBrakingDistance = 10.0;
        verticalSafetyDistance = 0.0;
        horizontalSafetyDistancel = 0.0;

        giveUpDistance = -4;
        nReliableCount = 2;

        enableHeadingSmoothing = false;
        enableSwerving = false;
        enableFollowing = false;
        enableTrafficLightBehavior = false;
        enableLaneChange = false;
        enableStopSignBehavior = false;
        enabTrajectoryVelocities = false;
        minIndicationDistance = 15;
    }
};

class CAR_BASIC_INFO {
public:
    double wheel_base;
    double length;
    double width;

    CAR_BASIC_INFO()
    {
        wheel_base = 2.7;
        length = 4.3;
        width = 1.82;
    }
};

int getNextClosePointIndex(const vector<UtilityNS::WayPoint> &trajectory,
                           const UtilityNS::WayPoint &curr_pos,
                           const int &prevIndex = 0);

double cast_from_0_to_2PI_Angle(const double &ang);

double diffBetweenTwoAngle(const double &a1, const double &a2);

void visualLaneInRviz(const vector<UtilityNS::WayPoint> &lane, ros::Publisher pub_testLane);

bool getRelativeInfo(const vector<UtilityNS::WayPoint> &trajectory,
                     const UtilityNS::WayPoint &p,
                     UtilityNS::RelativeInfo &info);

} // namespace UtilityNS


namespace RolloutGeneratorNS {

#define LANE_CHANGE_SPEED_FACTOR 0.5

class RolloutGenerator {
    
private:
    UtilityNS::WayPoint current_pose;
    UtilityNS::WayPoint init_pose;
    vector<vector<UtilityNS::WayPoint>> globalPaths;
    vector<UtilityNS::WayPoint> centralTrajectorySmoothed;
    vector<vector<UtilityNS::WayPoint>> globalPathSections;
    vector<vector<vector<UtilityNS::WayPoint>>> rollOuts;

    bool currentPose_flag;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Publisher pub_localTrajectoriesRviz;
    ros::Publisher pub_testLane;

    ros::Publisher pub_global_path;
    ros::Publisher pub_center_path;
    ros::Publisher pub_remaining_path;

    UtilityNS::PlanningParams PlanningParams;
    UtilityNS::CAR_BASIC_INFO CarInfo;
    double speed;

    void getGlobalPlannerPath_cb(nav_msgs::Path msg);

    void msgLane2LocalLane(nav_msgs::Path msg_path, vector<UtilityNS::WayPoint>& path);

    double calcAngleAndCost(vector<UtilityNS::WayPoint>& path);

    void extractPartFromTrajectory(const vector<UtilityNS::WayPoint>& originalPath,
        const double& waypointDensity, vector<UtilityNS::WayPoint>& extractedPath);

    void fixPathDensity(vector<UtilityNS::WayPoint>& path, const double& pathDensity);
    nav_msgs::Path fixPathDensity(nav_msgs::Path path);

    void trajectoryToMarkers(const vector<vector<vector<UtilityNS::WayPoint>>>& paths, visualization_msgs::MarkerArray& markerArray);

    void generateRunoffTrajectory(const vector<vector<UtilityNS::WayPoint>>& referencePaths,
        const double& speed, const double& carTipMargin, const double& rollInMargin,
        const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
        const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
        const double& SmoothTolerance, vector<vector<vector<UtilityNS::WayPoint>>>& rollOutsPaths);

    void calculateRollInTrajectories(const double& speed, const vector<UtilityNS::WayPoint>& originalCenter,
        vector<vector<UtilityNS::WayPoint>>& rollInPaths,
        const double& carTipMargin, const double& rollInMargin, const double& rollInSpeedFactor,
        const double& pathDensity, const double& rollOutDensity, const int& rollOutNumber,
        const double& SmoothDataWeight, const double& SmoothWeight, const double& SmoothTolerance);

    void smoothPath(vector<UtilityNS::WayPoint>& path, double weight_data, double weight_smooth, double tolerance);

public:
    RolloutGenerator();

    ~RolloutGenerator();

    nav_msgs::Path getCenterPath(vector<UtilityNS::WayPoint> pathIn);
    vector<nav_msgs::Path> getOtherpaths(vector<vector<UtilityNS::WayPoint>> pathsIn);
    nav_msgs::Path getRemainingPath(nav_msgs::Path pathIn, nav_msgs::Path centerPath);
    nav_msgs::Path calculatePathYaw(nav_msgs::Path pathIn);

    void run(nav_msgs::Path pathMsg, vector<nav_msgs::Path>& alternativePaths);

    void initROS();

    tf::StampedTransform transform;
};

} // namespace RolloutGeneratorNS
