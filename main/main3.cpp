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

    vector<double> ang(2);
    ang[0] = 30; //ALPHA
    ang[1] = 0; //BETA


    ofstream data("/home/humasoft/Soft-Arm/graphs/grandes.csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(20,20);
    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(20,20);

    SocketCanPort pm3("can1");
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(20,20);

    double radio=0.0093;
    vector<double> v_lengths(3);
    double posan1, posan2, posan3;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    IPlot probe(dts,"Plot Pitch");
    IPlot probe1(dts,"Plot Input");
    IPlot probe2(dts,"Plot L1");
    IPlot probe3(dts,"Plot L2");
    IPlot probe4(dts,"Plot L3");

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL

    cs[0]=ang[0];
    cs[1]=ang[1];


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;
    cout<<"Moving to Pitch: "+to_string(int(ang[0]))+ " and Yaw: "+to_string(int(ang[1]))<<endl;

    double interval=50; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        cs[0]=10*sin(t*5);
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(cs[0]);

        v_lengths[0]=0.001*( cs[0] / 1.5);
        //v_lengths[1]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );
        //v_lengths[2]=0.001*( (cs[0] / -3) - (cs[1] / 1.732) );

        // INVERTIDO
        v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) ); //Antiguo tendon 3
        v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) ); //Antiguo tendon 2



        posan1=(v_lengths[0])/radio;
        posan2=(v_lengths[1])/radio;
        posan3=(v_lengths[2])/radio;


        probe2.pushBack(m2.GetPosition());
        probe3.pushBack(m2.GetVelocity());
        probe4.pushBack(posan2);


        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);

    }

    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    probe4.Plot();

    m3.SetPosition(0);
    m2.SetPosition(0);
    m1.SetPosition(0);
    //sleep(4);
}
