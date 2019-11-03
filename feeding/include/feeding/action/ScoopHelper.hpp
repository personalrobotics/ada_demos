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

// add max_size property and anato vector
class myvec : public std::vector<double>
{
public:
    
    myvec(int maxsize): maxsize(maxsize){};
    // myvec(int maxsize)
    // {
    //     this->maxsize = maxsize;
    // };
    myvec(void) {};
    ~myvec(void) {};

    void push_back(double value)
    {
        if (this->size() < this->maxsize)
            std::vector<double>::push_back(value);
        if (this->size() == this->maxsize)
        {
            std::vector<double>::iterator k = this->begin();
            std::vector<double>::erase(k);
            std::vector<double>::push_back(value);
        }
    }

    // set max size of the vector
    void setMaxSize(int maxsize)
    {
        this->maxsize = maxsize;
    }

    int max_size()
    {
        return maxsize;
    }

    void analysis(double& mean, double& stdev)
    {
        double sum = std::accumulate(this->begin(), this->end(), 0.0);
        mean =  sum / this->size(); //mean

        double accum  = 0.0;
        std::for_each (this->begin(), this->end(), [&](const double d) {
            accum  += (d-mean)*(d-mean);
        });

        stdev = sqrt(accum/(this->size()-1)); //standard deviation
    }

private:

    int maxsize = 0;
};

class MarkersListener
{
private:    
    
    int seq;
    int maxsize = 30;
    visualization_msgs::Marker marker;
    Eigen::Isometry3d FoodTransform; // transformation from camera frame to food item frame
    double tx, ty, tz, rx, ry, rz, rw;
    myvec buffX, buffY, buffZ;
    double x, y, z;
    double sdx, sdy, sdz;
    Eigen::Isometry3d  plateCenter; 
    Eigen::Vector3d quadrantVec;

public:

    // MarkersListener(ros::NodeHandle n) 
    MarkersListener(ros::NodeHandle mNodeHandle) 
    {
        seq = 0;
        std::cout << "Contruct a MarkerArray Listener" << std::endl;
        buffX.setMaxSize(maxsize);
        buffY.setMaxSize(maxsize);
        buffZ.setMaxSize(maxsize);
        plateCenter = createIsometry(getRosParam<std::vector<double>>("/plate/centerPose", mNodeHandle));
        // sub = n.subscribe("/food_detector/marker_array", 100, callback);
    }

    void callback(visualization_msgs::MarkerArray marker_array) 
    {
        // std::cout << "call back" << std::endl;
        // if (sizeof(marker_array) == 0)
        // return;
        if (marker_array.markers.empty())
        {   
            return;
        }

        marker = marker_array.markers[0]; 
        // std::cout <<"Marker is in "<< marker.header.frame_id.c_str() << std::endl;
        tx = marker.pose.position.x; 
        ty = marker.pose.position.y;
        tz = marker.pose.position.z; 
        rx = marker.pose.orientation.x;
        ry = marker.pose.orientation.y;
        rz = marker.pose.orientation.z;
        rw = marker.pose.orientation.w; 

        buffX.push_back(tx);
        buffY.push_back(ty);
        buffZ.push_back(tz);

        buffX.analysis(x, sdx);
        buffY.analysis(y, sdy);
        buffZ.analysis(z, sdz);

        // std::cout << "tx = " << tx << ", ty = " << ty << ", tz = " << tz << std::endl;
        // std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;

        // quickly remove rotation.
        rx = 0; 
        ry = 0;
        rz = 0;
        rw = 1;

        // if (seq == 0) 
        // {
        //     PrintInfo("food", FoodTransform)
        // }

        Eigen::Quaterniond q(rw, rx, ry, rz);
        FoodTransform.translation() = Eigen::Vector3d(x, y, z); 
        FoodTransform.linear() = q.matrix();
        Eigen::Matrix3d m;
        m << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        // std::cout << m;
        FoodTransform.linear() = m;

        quadrantVec = FoodTransform.translation() - plateCenter.translation();

        seq++;
    }

    Eigen::Isometry3d getTransform() 
    {
        return FoodTransform;
    }

    Eigen::Isometry3d getValidTransform()
    {
        while (!isValid())
        {
            std::cout << "no" << std::endl;
        }

        return FoodTransform;
    }


    int getQuadrant1()
    {
        // int quadrant;
        double x, y;
        x = quadrantVec[0];
        y = quadrantVec[1];
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
        std::cout << "dx = " << quadrantVec[0] << ", dy = " << quadrantVec[1] << std::endl;
        double theta = atan2(quadrantVec[1], quadrantVec[0]);
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
        std::cout << "dx = " << quadrantVec[0] << ", dy = " << quadrantVec[1] << std::endl;
        return atan2(quadrantVec[1], quadrantVec[0]);
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
        return quadrantVec;
    }

    bool isValid()
    {
        // double tx = FoodTransform.translation()[0];
        // double ty = FoodTransform.translation()[1];
        // double tz = FoodTransform.translation()[2];
        if (seq < maxsize)
        {   
            // std::cout << "seq" << seq << std::endl;
            return false;
        }
        // empirical range for food, food should be within the plate
        std::cout << "tx = " << x << ", ty = " << y << ", z = " << tz << std::endl;
        if (x > 0.35 || x < 0.18)
            return false;
        if (y > -0.20 || y < -0.38)
            return false;
        if (z > 0.3 || z < 0.22)
            return false;

        std::cout << "Find Valid Pose" << std::endl;
        return true;
    } 

    visualization_msgs::Marker getMarker()
    {
        return marker;
    }

};
