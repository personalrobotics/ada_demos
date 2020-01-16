#include <numeric>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <libada/util.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;
using ada::util::getRosParam;
using ada::util::createIsometry;

class ScoopHelper
{
private:    
    
    Eigen::Isometry3d mFoodTransform; // transformation from camera frame to food item frame
    Eigen::Isometry3d  mPlateCenter; 
    Eigen::Vector3d mQuadrantVec;

public:

    // MarkersListener(ros::NodeHandle n) 
    ScoopHelper(ros::NodeHandle mNodeHandle,
                Eigen::Isometry3d FoodTransform) 
    {
        std::cout << "Contruct a ScoopHelper" << std::endl;
        mFoodTransform = FoodTransform;
        mPlateCenter = createIsometry(getRosParam<std::vector<double>>("/plate/centerPose", mNodeHandle));
        mQuadrantVec = mFoodTransform.translation() - mPlateCenter.translation();
        // try to use removeRotation function in feeding/util.cpp ? there's probably some C++ convention?
        
        // std::cout << "before mFoodTransform = " << mFoodTransform << std::endl;
        Eigen::Matrix3d m;
        m << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        // std::cout << m;
        mFoodTransform.linear() = m;
        // std::cout << "after mFoodTransform = " << mFoodTransform << std::endl;
    }

    Eigen::Isometry3d getTransform() 
    {
        return mFoodTransform;
    }

    Eigen::Isometry3d getValidTransform()
    {
        while (!isValid())
        {
            std::cout << "no" << std::endl;
        }

        return mFoodTransform;
    }


    int getQuadrant1()
    {
        // int quadrant;
        double x, y;
        x = mQuadrantVec[0];
        y = mQuadrantVec[1];
        if (x > 0 && y > 0)
            return 1;
        if (x < 0 && y > 0)
            return 2;
        if (x < 0 && y < 0)
            return 3;
        if (x > 0 && y < 0)
            return 4;
    }

    // My own defined Quadrant.
    int getQuadrant2()
    {
        std::cout << "dx = " << mQuadrantVec[0] << ", dy = " << mQuadrantVec[1] << std::endl;
        double theta = atan2(mQuadrantVec[1], mQuadrantVec[0]);
        std::cout << "atan2 , theta = " << theta << std::endl;

        if (theta >= M_PI/4.0 && theta < M_PI*3/4.0)
            return 1;
        if (theta >= M_PI*3/4.0 || theta < -M_PI*3/4.0)
            return 2;
        if (theta >= -M_PI*3/4.0 && theta < -M_PI/4.0)
            return 3;
        if (theta >= -M_PI/4.0 && theta < M_PI/4.0)
            return 4;
    }

    double getTheta1()
    {
        std::cout << "dx = " << mQuadrantVec[0] << ", dy = " << mQuadrantVec[1] << std::endl;
        return atan2(mQuadrantVec[1], mQuadrantVec[0]);
    }

    double getTheta2()
    {
        int quadrant = getQuadrant2();
        std::cout << "quadrant = " << quadrant << std::endl;
        if (quadrant == 1)
            return M_PI/2.0;
        if (quadrant == 2)
            return 0;
        if (quadrant == 3)
            return M_PI/2.0;
        if (quadrant == 4)
            return 0;
    }

    double getDirection2()
    {
        int quadrant = getQuadrant2();
        if (quadrant == 1 || quadrant == 4)
            return 1.0;
        if (quadrant == 2 || quadrant == 3)
            return -1.0;
    }

    Eigen::Vector3d getQuadrantVec()
    {
        return mQuadrantVec;
    }

    bool isValid()
    {
        double x = mFoodTransform.translation()[0];
        double y = mFoodTransform.translation()[1];
        double z = mFoodTransform.translation()[2];
        // empirical range for food, food should be within the plate
        std::cout << "tx = " << x << ", ty = " << y << ", z = " << z << std::endl;
        if (x > 0.35 || x < 0.18)
            return false;
        if (y > -0.20 || y < -0.38)
            return false;
        if (z > 0.3 || z < 0.22)
            return false;

        std::cout << "Find Valid Pose" << std::endl;
        return true;
    } 
};
