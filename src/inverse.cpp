#include "manipulability/inverse.h"

namespace manipulability {

Eigen::Matrix<double, 6, 7> calcJacobian(const Eigen::VectorXd &joint_position) {
    // ここにヤコビ行列計算の実装を書く
    //double dbs = 340, dse = 400, dew = 400, dwf = 126;
    double dbs = 0.34, dse = 0.4, dew = 0.4, dwf = 0.126;
    double th1 = joint_position[0], th2 = joint_position[1], th3 = joint_position[2], th4 = joint_position[3], th5 = joint_position[4], th6 = joint_position[5], th7 = joint_position[7];

    double J11 = dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - dse * sin(th1) * sin(th2);
    double J12 = dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) + dse * cos(th1) * sin(th2);
    double J13 = 0;
    double J14 = 0;
    double J15 = 0;
    double J16 = 1;

    Eigen::Matrix<double, 6, 1> J1;
    J1 << J11, J12, J13, J14, J15, J16;

    double J21 = cos(th1) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))) + dse * cos(th2));
    double J22 = sin(th1) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))) + dse * cos(th2));
    double J23 = sin(th1) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - dse * sin(th1) * sin(th2)) - cos(th1) * (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) + dse * cos(th1) * sin(th2));
    double J24 = -sin(th1);
    double J25 = cos(th1);
    double J26 = 0;

    Eigen::Matrix<double, 6, 1> J2;
    J2 << J21, J22, J23, J24, J25, J26;

    double J31 = cos(th2) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - dse * sin(th1) * sin(th2)) + sin(th1) * sin(th2) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))) + dse * cos(th2));
    double J32 = cos(th2) * (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) + dse * cos(th1) * sin(th2)) - cos(th1) * sin(th2) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))) + dse * cos(th2));
    double J33 = -sin(th1) * sin(th2) * (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) + dse * cos(th1) * sin(th2)) - cos(th1) * sin(th2) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - dse * sin(th1) * sin(th2));
    double J34 = cos(th1) * sin(th2);
    double J35 = sin(th1) * sin(th2);
    double J36 = cos(th2);

    Eigen::Matrix<double, 6, 1> J3;
    J3 << J31, J32, J33, J34, J35, J36;

    double J41 = -(cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)))) - sin(th2) * sin(th3) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)));
    double J42 = -(cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)))) - sin(th2) * sin(th3) * (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)));
    double J43 = (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2))) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)) - (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)));
    double J44 = cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3);
    double J45 = cos(th2) * sin(th1) * sin(th3) - cos(th1) * cos(th3);
    double J46 = -sin(th2) * sin(th3);

    Eigen::Matrix<double, 6, 1> J4;
    J4 << J41, J42, J43, J44, J45, J46;

    double J51 = (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2))) - (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))));
    double J52 = (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2))) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) - (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) * (dew * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)) + dwf * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4))));
    double J53 = (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) * (dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) + dew * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2))) - (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) * (dwf * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) + dew * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)));
    double J54 = sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2);
    double J55 = cos(th4) * sin(th1) * sin(th2) - sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1));
    double J56 = cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4);

    Eigen::Matrix<double, 6, 1> J5;
    J5 << J51, J52, J53, J54, J55, J56;

    double J61 = -dwf * (sin(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) - cos(th5) * sin(th2) * sin(th3)) * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)))) - dwf * (sin(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) - cos(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3))) * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)));
    double J62 = -dwf * (sin(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) - cos(th5) * sin(th2) * sin(th3)) * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) - dwf * (sin(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) - cos(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3))) * (sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4)));
    double J63 = dwf * (cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)))) * (sin(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) - cos(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3))) - dwf * (sin(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) - cos(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3))) * (cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3))));
    double J64 = sin(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) - cos(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3));
    double J65 = cos(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3)) - sin(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4));
    double J66 = cos(th5) * sin(th2) * sin(th3) - sin(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2));

    Eigen::Matrix<double, 6, 1> J6;
    J6 << J61, J62, J63, J64, J65, J66;

    double J71 = 0;
    double J72 = 0;
    double J73 = 0;
    double J74 = cos(th6) * (sin(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) + cos(th1) * cos(th4) * sin(th2)) - sin(th6) * (cos(th5) * (cos(th4) * (sin(th1) * sin(th3) - cos(th1) * cos(th2) * cos(th3)) - cos(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th3) * sin(th1) + cos(th1) * cos(th2) * sin(th3)));
    double J75 = sin(th6) * (cos(th5) * (cos(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) + sin(th1) * sin(th2) * sin(th4)) + sin(th5) * (cos(th1) * cos(th3) - cos(th2) * sin(th1) * sin(th3))) - cos(th6) * (sin(th4) * (cos(th1) * sin(th3) + cos(th2) * cos(th3) * sin(th1)) - cos(th4) * sin(th1) * sin(th2));
    double J76 = sin(th6) * (cos(th5) * (cos(th2) * sin(th4) - cos(th3) * cos(th4) * sin(th2)) + sin(th2) * sin(th3) * sin(th5)) + cos(th6) * (cos(th2) * cos(th4) + cos(th3) * sin(th2) * sin(th4));

    Eigen::Matrix<double, 6, 1> J7;
    J7 << J71, J72, J73, J74, J75, J76;

    Eigen::Matrix<double, 6, 7> Jacobi;
    Jacobi << J1, J2, J3, J4, J5, J6, J7;

    return Jacobi;
}

Eigen::Matrix<double, 7, 6> calcJacobianInverse(const Eigen::MatrixXd &jacobian) {
    // 逆行列計算
    return jacobian.completeOrthogonalDecomposition().pseudoInverse();
}

}  // namespace manipulability
