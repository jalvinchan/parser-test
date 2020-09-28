#include <iostream>
#include "calibration_parser.hpp"

using namespace std;

int main(){

    CalibrationParser parser("../camchain-2020-08-26-17-42-05.yaml");
    auto& calib = parser.getCalibrations();
    
    for(auto& c:calib){
        cout << c.camera_model << endl;
        cout << c.intrinsics << endl;
        cout << c.distortion_model << endl;
        cout << c.distortion_coeffs << endl;
        // cout << c.T_w_c << endl;
        cout << c.T_cn_cnm1 << endl;
    }

    return 0;
}
