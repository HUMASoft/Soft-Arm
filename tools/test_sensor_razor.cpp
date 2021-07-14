#include "IMURazor9DOF.h"
#include "IPlot.h"
#include "math.h"
#include "fcontrol.h"

int main()
{


    IMURazor9DOF imu("/dev/ttyUSB1");

    vector<double> data;
    double dts=0.02;
    SamplingTime Ts(dts);
    double pitch, roll, yaw;
    IPlot plPitch(dts,"Pitch");
    IPlot plYaw(dts,"Yaw");
    IPlot plRoll(dts,"Roll");




    imu.GetYawPitchRoll(dts, yaw, pitch, roll );
    double offset_yaw = yaw;

    for (double t=0; t<20; t+=dts)
    {


        imu.GetYawPitchRoll(dts, yaw, pitch, roll);
        cout << "-> pitch: " << pitch << ", roll: " << roll << ", yaw: " << yaw << endl;
//        cout << imu.GetLine() << endl;
        //plPitch.pushBack(pitch);
        plYaw.pushBack(yaw-offset_yaw);
        plRoll.pushBack(-roll);


        //plPitch.pushBack(pitch*180/M_PI);
        //plYaw.pushBack(yaw*180/M_PI);
//        usleep(dts*1000*1000);

        Ts.WaitSamplingTime();
    }



    plPitch.Plot();
    plYaw.Plot();
    plRoll.Plot();

    return 0;
}



