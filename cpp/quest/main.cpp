//  Test case for QUEST algorithm
//  Rishav (2020/12/7)

#include <iostream>
#include "quest.h"

int main()
{
    quest qst(2);
    Quaternion q;
    Vector<3> ypr;

    // True DCM
    Matrix<3, 3> Q_truth({{0.5335, 0.8080, 0.2500},
                          {-0.8080, 0.3995, 0.4330},
                          {0.2500, -0.4330, 0.8660}});

    // Unit vectors in inertial frame
    Vector<3> compass_i({0.2673, 0.5345, 0.8018});
    Vector<3> gravity_i({-0.3124, 0.93705, 0.1562});
    
    // Unit measurement vectors
    Vector<3> compass_b;
    Vector<3> gravity_b;
    compass_b = Q_truth * compass_i;
    gravity_b = Q_truth * gravity_i;

    // QUEST algorithm
    q = qst.update(compass_i, gravity_i, compass_b, gravity_b);
    ypr = q.get_euler();

    std::cout << "True DCM:";
    std::cout << Q_truth;
    std::cout << "Computed DCM:";
    std::cout << q.get_dcm();

    return 0;
}