#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using namespace Eigen;

class Pose
{
public:

    double z, y, roll_angle;

    Pose(double z1, double y1, double roll_angle1) : z(z1), y(y1), roll_angle(roll_angle1) {}

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
