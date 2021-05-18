
#include <iostream>
#include <ios>

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <string.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/hex.hpp>
#include "imu3dmgx510.h"
#include "fcontrol.h"


#include <tuple>
//#include <yarp/os/Bottle.h>


//using namespace std;
using namespace boost::asio;
using namespace boost::algorithm;
using namespace std::string_literals;
using namespace stateestimation;
using std::cin;
using std::cout;




int main()
{
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;

    misensor.set_streamon();
    double dts=1/freq;
    double *EulerAngles;
    SamplingTime Ts(dts);


    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;

//        EulerAngles = misensor.EulerAngles();
//        cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2] << endl;
    }

    misensor.Reset();
    sleep(2);

    for (double t=0;t<1;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;

//        EulerAngles = misensor.EulerAngles();
//        cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2] << endl;
    }
    return 0;
}
