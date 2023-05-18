//main/main_identificador.cpp

#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"
#include <complex>
#include <math.h>

//
#include "imu3dmgx510.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <vector>


int main ()
{

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Identificacion/Identification_Maria/prueba_2/test.csv",std::ofstream::out); // /home/humasoft/code/graficas

    double interval= 10; // experiment seconds.

    vector<double> ang(2);
    ang[0] = 50; //ALPHA
    ang[1] =50; //BETA
    //ang[1]=ang[1]/2;

       //--Can port communications--

    string can = "can0";
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
    IPlot probe2(dts,"Plot Input");
    IPlot probe3(dts,"Plot Id");


    vector<double> cs(2); //CONTROL SIGNAL
    vector<double> csr(2); //CONTROL SIGNAL AUX


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<1;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;



            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);

            for (double t=0;t<1; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);
                probe.pushBack(pitch*180/M_PI);
                data <<0 << " , " <<0<< " , " <<0 << " , " <<0<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR

                Ts.WaitSamplingTime();
            }
            double A=15; //input amplitude
            double w=0; //velocity in rad/seconds
            double ptarg;
            double ytarg=0;
            for (double t=0;t<interval; t+=dts)
            {
                for (w; w<3; w+=0.01){
                ptarg= A*sin(w*t);

                //decoupling
                v_lengths[0]=0.001*( ptarg / 1.5);
                v_lengths[1]=0.001*( - (ptarg / 3) - (ytarg / 1.732) ); //Antiguo tendon 3
                v_lengths[2]=0.001*( (ytarg / 1.732) - (ptarg / 3) ); //Antiguo tendon 2

                posan1=(v_lengths[0])/radio;
                posan2=(v_lengths[1])/radio;
                posan3=(v_lengths[2])/radio;

                m1.SetPosition(posan1);
                m2.SetPosition(posan2);
                m3.SetPosition(posan3);

                misensor.GetPitchRollYaw(pitch,roll,yaw);
                cout<< "posan1:"<<posan1<< ";   posan2:"<<posan2 <<endl;


                data <<ptarg << " , " <<ytarg<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
                //cout << endl;
                Ts.WaitSamplingTime();
                 }
            }
            cout <<"Done:"<<endl;
            cout<< "Alpha:"<<ptarg<< ";   Beta:"<<ytarg <<endl;

            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);

            for (double t=0;t<interval; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);
                probe.pushBack(pitch*180/M_PI);
                data <<0 << " , " <<0<< " , " <<0 << " , " <<0<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR

                Ts.WaitSamplingTime();
            }



    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
//    sleep(4);


    return 0;
}





