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

const char *PORT = "/dev/ttyUSB0";
//


int main ()
{


    ofstream data("/home/humasoft/code/Soft-Arm/graphs/desacoplado50-10V3.csv",std::ofstream::out); // /home/humasoft/code/graficas
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


    //Controller

    FPDBlock conP(0.27,1.492,-0.9,dts); //(kp,kd,exp,dts) 0.0214437
    FPDBlock reset(conP); //Used for control reset

    //TableArmKinematics a("../Tabla170.csv");

    double radio=0.0093;

    vector<double> v_lengths(3);
    vector<double> ang(2);
    double posan1, posan2, posan3;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;
    //    double *EulerAngles;

    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {


        misensor.GetPitchRollYaw(pitch,roll,yaw);
        cout<<"Calibrando"<<endl;
        cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }

    vector<double> ierror(2);
    vector<double> cs(2);

    ang[0] = -50; //ALPHA
    ang[1] = 0; //BETA



    for (long stops = 4; stops > 0 ; stops--)
    {

        double interval=5; //in seconds
        for (double t=0;t<interval; t+=dts)
        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);

            //negative feedback
            ierror[0] = ang[0] - pitch*M_PI/180;

            //ierror= ierror*M_PI/180; //degrees to rad

            //controller computes control signal
            cs[0] = ierror[0] > conP;
            //  cs = ierror > intcon;

            if (!isnormal(cs[0])) cs[0] = 0;


            v_lengths[0]=0.001*( cs[0] / 1.5);
            v_lengths[1]=0.001*( (ang[1] / 1.732) - (cs[0] / 3) );
            v_lengths[2]=0.001*( (cs[0] / -3) - (ang[1] / 1.732) );

            posan1=(v_lengths[0])/radio;
            posan2=(v_lengths[1])/radio;
            posan3=(v_lengths[2])/radio;

            m1.SetPosition(posan1);
            m2.SetPosition(posan2);
            m3.SetPosition(posan3);

            //cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
            //cout << "Length variation" << endl;
            //cout << "1: " << v_lengths[0]  << "2: " << v_lengths[1] << "3: " << v_lengths[2] <<endl;
            // Sensor data
            //misensor.GetPitchRollYaw(pitch,roll,yaw);
            data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
            //cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI<< endl; //CR
            //cout << endl;
        }

        conP = FPDBlock(reset); //Reset?

        ang[0]+=25;
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
        sleep(4);


    }
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    sleep(4);



}

