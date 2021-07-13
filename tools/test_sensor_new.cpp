//Used libs in the project

#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"

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

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;

    //misensor.set_streamon();
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //double *EulerAngles;

    for (double t=0;t<100;t+=dts)
    {


        misensor.GetPitchRollYaw(pitch,roll,yaw);
        cout<<"Calibrando"<<endl;
        cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }




//        EulerAngles = misensor.EulerAngles();
//        cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2] << endl;

    return 0;
}






