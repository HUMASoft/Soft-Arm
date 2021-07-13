#include "IMURazor9DOF.h"
#include "IPlot.h"
#include "math.h"
#include "fcontrol.h"

int main()
{


    IMURazor9DOF imu("/dev/ttyUSB0");

    vector<double> data;
    double dts=0.02;
    SamplingTime Ts(dts);
    double pitch, roll, yaw;
    IPlot plPitch(dts,"Pitch");
    IPlot plYaw(dts,"Yaw");
    IPlot plZ(dts,"Z");




    imu.GetPitchRollYaw(dts, pitch, roll, yaw );

    for (double t=0; t<10; t+=dts)
    {
//        imu.GetPitchRollYaw(dts, pitch, roll, yaw );
        cout << "-> pitch: " << pitch*180/M_PI << ", roll: " << roll*180/M_PI << ", yaw: " << yaw*180/M_PI << endl;
//        cout << imu.GetLine() << endl;
        imu.GetRawData(data);
        plPitch.pushBack(data[6]);
        plYaw.pushBack(data[7]);
        plZ.pushBack(data[8]);


        //plPitch.pushBack(pitch*180/M_PI);
        //plYaw.pushBack(yaw*180/M_PI);
//        usleep(dts*1000*1000);

        Ts.WaitSamplingTime();
    }



    plPitch.Plot();
    plYaw.Plot();
    plZ.Plot();

    return 0;
}



