//main/main_identificador.cpp

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
    ang[0] = 50; //ALPHA
    ang[1] =50; //BETA
    //ang[1]=ang[1]/2;

       //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(3,3);

    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(3,3);

    SocketCanPort pm3("can1");
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


    //identification
    ulong numOrder=0,denOrder=2;

    OnlineSystemIdentification model(numOrder, denOrder );

    vector<double> num(numOrder+1),den(denOrder+1); //(order 0 also counts)
    SystemBlock sys(num,den); //the resulting identification

    double gain;

    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;

    double interval=10; //in seconds
    string sNum="";
    string sDen="";


    for (cs[0] = -ang[0] ; cs[0] <= ang[0] ; cs[0]= cs[0]+10)
    {
        for (cs[1] = -ang[1] ; cs[1] <= ang[1] ; cs[1]= cs[1]+10)
        {

            ofstream data("/home/humasoft/code/Soft-Arm/graphs/Identificacion/Indentificacion_P"+to_string(int(cs[0]))+"_Y"+to_string(int(cs[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas

            for (double t=0;t<interval; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);

                model.UpdateSystem(cs[0], pitch*180/M_PI);

                model.GetSystemBlock(sys);
                gain=sys.GetZTransferFunction(num,den);

                v_lengths[0]=0.001*( cs[0] / 1.5);
                v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) ); //Antiguo tendon 3
                v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) ); //Antiguo tendon 2

                posan1=(v_lengths[0])/radio;
                posan2=(v_lengths[1])/radio;
                posan3=(v_lengths[2])/radio;

                m1.SetPosition(posan1);
                m2.SetPosition(posan2);
                m3.SetPosition(posan3);

                data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << " , " << cs[0] << " , " <<cs[1] << endl; //CR
                //cout << endl;
                Ts.WaitSamplingTime();
            }
            cout <<"Done"<<endl;
            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);

            sNum="";
            sDen="";

            for(int i = 0; i < num.size(); i++)
                {
                    sNum=sNum+ to_string(gain*num[i])+", ";
                }



            for(int i = den.size()-1; i>=0; i--)
                {
                    sDen=sDen+", "+ to_string(den[i]);
                }

            cout <<sNum+sDen<<endl;
            data << sNum+sDen<<endl;
            sleep(4);
        }

    }


    //sys.PrintZTransferFunction(dts);

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
//    sleep(4);
}

