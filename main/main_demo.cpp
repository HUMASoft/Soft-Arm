#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"
#include "imu3dmgx510.h"


int main ()
{

    //-- Inicialization--
    vector<double> ang(2);
    vector<double> v_lengths(3);

    double posan1, posan2, posan3;
    double pitch, roll, yaw;

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL

    double radio=0.0093;

    //--Can port communications--
    string can = "can1";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(3,3);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(3,3);

    SocketCanPort pm3(can);
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(3,3);


    // SENSOR
    cout<<"Can Set up1"<<endl;
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //plot
//    IPlot probe(dts,"Plot Pitch");
//    IPlot probe1(dts,"Plot Yaw");
//    IPlot probe2(dts,"Plot CPitch");
//    IPlot probe3(dts,"Plot m2");
//    IPlot probe4(dts,"Plot m3");

     cout<<"Can Set up2"<<endl;

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();
int i=0;
    for (double t=0;t<3;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;


    ang[0] = 40; //ALPHA
    ang[1] = 40; //BETA

    // 4 axis poses
    vector<double> valores(8);
    valores[0]=0;
    valores[1]=ang[1];
    valores[2]=0;
    valores[3]=-ang[1];
    valores[4]=-ang[0];
    valores[5]=0;
    valores[6]=ang[0]+40;
    valores[7]=0;

    // Test duration
    double interval_time; //in seconds
    double num_loops=1; //Number of loops


    for (int loop=0;loop<num_loops; loop+=1){ //

        for (uint iter_pose=0; iter_pose < valores.size(); iter_pose=iter_pose+2)
        {
            cs[0]=valores[iter_pose];
            cs[1]=valores[iter_pose+1];
            cout<<"Moving to Input Pitch: "+to_string(int(cs[0]))+ " and Yaw: "+to_string(int(cs[1]))<<endl;

            // Moving arm to pose
            interval_time=5;
            for (double t=0;t<interval_time; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);

//                probe.pushBack(pitch*180/M_PI);
//                probe1.pushBack(yaw*180/M_PI);
//                //            probe2.pushBack(cs[0]);
//                //            probe3.pushBack(cs[1]);
//                probe2.pushBack(m2.GetPosition());
//                probe3.pushBack(m2.GetVelocity());

                if (!isnormal(cs[0])) cs[0] = 0;

                if (!isnormal(cs[1])) cs[1] = 0;


                v_lengths[0]=0.001*( cs[0] / 1.5);
                v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
                v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );


                posan1=(v_lengths[0])/radio;
                posan2=(v_lengths[1])/radio;
                posan3=(v_lengths[2])/radio;

                m1.SetPosition(posan1);
                m2.SetPosition(posan2);
                m3.SetPosition(posan3);

                Ts.WaitSamplingTime();
            }

            // Moving arm to zero
            interval_time=3;
            for (double t=0;t<interval_time; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);

//                probe.pushBack(pitch*180/M_PI);
//                probe1.pushBack(yaw*180/M_PI);
//                //            probe2.pushBack(0);
//                //            probe3.pushBack(0);
//                probe2.pushBack(m2.GetPosition());
//                probe3.pushBack(m3.GetPosition());

                m1.SetPosition(0);
                m2.SetPosition(0);
                m3.SetPosition(0);
                Ts.WaitSamplingTime();
            }
        }
    }

    cout <<"Done" << endl;
    //probe.Plot();
    //probe1.Plot();
    //probe2.Plot();
    //probe3.Plot();
    //probe4.Plot();

}
