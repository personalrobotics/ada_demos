#include <numeric>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;

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
    int maxsize = 100;
    visualization_msgs::Marker marker;
    Eigen::Isometry3d FoodTransform; // transformation from camera frame to food item frame
    double tx, ty, tz, rx, ry, rz, rw;
    myvec buffX, buffY, buffZ;
    // myvec buffX = new myvec(100);
    // myvec buffY = new myvec(100);
    // myvec buffZ = new myvec(100);
    double x, y, z;
    double sdx, sdy, sdz;

public:

    //MarkersListener(ros::NodeHandle n) 
    MarkersListener() 
    {
        seq = 0;
        std::cout << "Contruct a MarkerArray Listener" << std::endl;
        buffX.setMaxSize(maxsize);
        buffY.setMaxSize(maxsize);
        buffZ.setMaxSize(maxsize);
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

    bool isValid() 
    {
        // empirical range for food, food should be within the plate
        std::cout << "tx = " << tx << ", ty = " << ty << ", tz = " << tz << std::endl;

        if (tx > 0.35 || tx < 0.18)
            return false;
        if (ty > -0.20 || ty < -0.38)
            return false;
        if (tz > 0.3 || tz < 0.22)
            return false;

        std::cout << "Find Valid Pose" << std::endl;
        return true;
    }

    visualization_msgs::Marker getMarker()
    {
        return marker;
    }

};

class Pose
{
public:

    double z, y, roll_angle;

    Pose(double z1, double y1, double roll_angle1) : z(z1), y(y1), roll_angle(roll_angle1) {};

};

class PolyTraj
{
private:

    double h, d;
    double a[4];

    void solve()
    {
        Matrix4d A;
        Vector4d b;
        A << 1,0,0,0, 0,d,d*d,d*d*d, 0,1,2*d,3*d*d, 0,1,4*d,12*d*d;
        b << 0, h, 0, 0;
        Vector4d x = A.colPivHouseholderQr().solve(b);
        for(int i=0; i<4; i++) {
            a[i] = x[i];
        }
    }

    double func(double z, int n =4)
    {
        double d = 0;
        for (int i = 0; i < n; i++)
            d += pow(z, i) * a[i];
        return d;
    }

    double der1(double z, int n = 4)
    {
        double d = 0;
        for (int i = 1; i < n; i++)
            d += pow(z, i - 1) * i * a[i];
        return d;
    }

    double roll_angle(double z) {
        return atan(-der1(z));
    }

public:

    PolyTraj(double h1, double d1)
    {
        h = h1;
        d = d1;
        solve();
    }

    void printCoefficient()
    {
        std::cout<< "h: " << h << " d: " << d << std::endl;
        for(int i=0; i<(sizeof(a)/sizeof(*a)); i++)
            std::cout << a[i] << std::endl;
    }

    Pose eval(double z)
    {
        // std::cout << "z:" << z << std::endl;
        // std::cout << "y:" << func(z) << std::endl;
        // std::cout << "roll_angle:" << roll_angle(z) << std::endl;
        return Pose(z, func(z), roll_angle(z));
    }

    // simply discretize z axis, but discretize time might be a better idea. implement that later.
    // num: num of waypoints
    std::vector<Pose> getWayPoints(int num)
    {
        double step = 2*d / (double) num;
        std::vector<Pose> wayPoints;
        for (int i = 1; i <= num; i++)
        {
            wayPoints.push_back(eval(i*step));
        }
        return wayPoints;
    }

    void test()
    {
        std::cout << "result:" << func(0) << std::endl;
        std::cout << "der1:" << der1(0) << std::endl;
        std::cout << "roll_angle:" << roll_angle(0) << std::endl;
    }

};
