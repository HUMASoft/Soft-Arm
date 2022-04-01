#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"

//
#include "imu3dmgx510.h"


int main ()
{

    double posan1, posan2, posan3;

    posan1=119.27;
    posan2=-117.38;
    posan3=-0.37;

    double vel=3;

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Tests/position.csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--

    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(vel,3);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(vel,3);
    SocketCanPort pm3(can);
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(vel,3);



    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //plot
    IPlot probe(dts,"Plot Pitch ");
    IPlot probe1(dts,"Plot Yaw");
    IPlot probe2(dts,"Plot cs1");
    IPlot probe3(dts,"Plot cs2");
    IPlot probe4(dts,"Plot m2");

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<5;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;

    double interval=5; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);

        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);

        probe3.pushBack(m2.GetVelocity());
        probe4.pushBack(m2.GetPosition());

        m1.SetPosition(posan1*M_PI/180);
        m2.SetPosition(posan2*M_PI/180);
        m3.SetPosition(posan3*M_PI/180);
        data  <<posan1<< " , " <<posan2 << " , " <<posan3<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
        //cout << t <<endl;
        Ts.WaitSamplingTime();
    }
    cout <<"Done" << endl;
    //conP = FPDBlock(resetP); //Reset?
    //conY = FPDBlock(resetY); //Reset?

    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    probe4.Plot();


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
//    sleep(4);
}
