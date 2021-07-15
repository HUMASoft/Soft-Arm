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


    ofstream data("/home/humasoft/code/Soft-Arm/graphs/desacoplado_Vel10_50_V2.csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(10,10);

    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(10,10);

    SocketCanPort pm3("can1");
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(10,10);


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
//        misensor.GetPitchRollYaw(pitch,roll,yaw);
//        cout << "ROLL: " << roll*180/M_PI << " ; PITCH: "  << pitch*180/M_PI << " ; YAW: " << yaw*180/M_PI <<  endl;


        misensor.GetPitchRollYaw(pitch,roll,yaw);
        cout<<"Calibrando"<<endl;
        cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }



    ang[0] = 0; //ALPHA
    ang[1] = 0; //BETA

    // CHANGING ALPHA AND BETA
    for (ang[0] = -50 ; ang[0] <= 50 ; ang[0]= ang[0]+10)
    {
        for (ang[1] = -50 ; ang[1] <= 50 ; ang[1]= ang[1]+10)
        {
            v_lengths[0]=0.001*( ang[0] / 1.5);
            v_lengths[1]=0.001*( (ang[1] / 1.732) - (ang[0] / 3) );
            v_lengths[2]=0.001*( (ang[0] / -3) - (ang[1] / 1.732) );

            posan1=(v_lengths[0])/radio;
            posan2=(v_lengths[1])/radio;
            posan3=(v_lengths[2])/radio;

            m1.SetPosition(posan1);
            m2.SetPosition(posan2);
            m3.SetPosition(posan3);
            double interval=5; //in seconds
            for (double t=0;t<interval; t+=dts)
            {
                cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
                cout << "Length variation" << endl;
                cout << "1: " << v_lengths[0]  << "2: " << v_lengths[1] << "3: " << v_lengths[2] <<endl;
                // Sensor data
                misensor.GetPitchRollYaw(pitch,roll,yaw);
                data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
                cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI<< endl; //CR
                cout << endl;
            }


            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);

            for (double t=0;t<interval; t+=dts)
            {
                cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
                misensor.GetPitchRollYaw(pitch,roll,yaw);
                data <<0 << " , " <<0<< " , " << roll << " , " << pitch << " , " << yaw <<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
                cout<< "Vuelta"<<endl;
                cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI<< endl; //CR
                cout << endl;


                Ts.WaitSamplingTime();
            }
            misensor.Reset();
            sleep(2);

        }

    }

    m3.SetPosition(0);
    m2.SetPosition(0);
    m1.SetPosition(0);

}


//    // FIXED BETA


//    for (ang[0] = 0 ; ang[0] < 50 ; ang[0]= ang[0]+5)
//    {
//        double interval=2; //in seconds
//        for (double t=0;t<interval; t+=dts)
//        {
//            v_lengths[0]=0.001*( ang[0] / 1.5);
//            v_lengths[1]=0.001*( (ang[1] / 1.732) - (ang[0] / 3) );
//            v_lengths[2]=0.001*( (ang[0] / -3) - (ang[1] / 1.732) );

//            posan1=(v_lengths[0])/radio;
//            posan2=(v_lengths[1])/radio;
//            posan3=(v_lengths[2])/radio;

//            m1.SetPosition(posan1);
//            m2.SetPosition(posan2);
//            m3.SetPosition(posan3);

//            // MOTORS MUST BE TURNED ON
//            // Ponerlo en negativo (el motor va al reves)
//            cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
//            cout << "Length variation" << endl;
//            cout << "1: " << v_lengths[0]  << "2: " << v_lengths[1] << "3: " << v_lengths[2] <<endl;
//            // Sensor data
//            EulerAngles = misensor.EulerAngles();
//            data << EulerAngles[0] << " , " << EulerAngles[1] << " , " << EulerAngles[2]<< endl; //CR
//            cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2]<< endl; //CR
//            cout << endl;
//        }

//        ang[0] = ang[0] + 5; //ALPHA
//        m1.SetPosition(0);
//        m2.SetPosition(0);
//        m3.SetPosition(0);

//        for (double t=0;t<interval; t+=dts)
//        {

//            EulerAngles = misensor.EulerAngles();
//            data << EulerAngles[0] << " , " << EulerAngles[1] << " , " << EulerAngles[2]<< endl; //CRcout << endl;

//            cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2]<< endl; //CR
//            cout << endl;

//            Ts.WaitSamplingTime();
//        }
//        cout <<"Posicion 0"<< endl;
//        cout << "Calibrating IMU..." << endl;
//        misensor.calibrate();
//        cout << "Calibration done" << endl;
//    }


//    // FIXED ALPHA

//    cout<< "Empieza"<< endl;

//    for (ang[1] = -40 ; ang[1] <= 40; ang[1]= ang[1]+5)
//    {
//        double interval=2; //in seconds
//        for (double t=0;t<interval; t+=dts)
//        {
//            v_lengths[0]=0.001*( ang[0] / 1.5);
//            v_lengths[1]=0.001*( (ang[1] / 1.732) - (ang[0] / 3) );
//            v_lengths[2]=0.001*( (ang[0] / -3) - (ang[1] / 1.732) );

//            posan1=(v_lengths[0])/radio;
//            posan2=(v_lengths[1])/radio;
//            posan3=(v_lengths[2])/radio;

//            m1.SetPosition(posan1);
//            m2.SetPosition(posan2);
//            m3.SetPosition(posan3);

//            // MOTORS MUST BE TURNED ON
//            // Ponerlo en negativo (el motor va al reves)
//            cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
//            cout << "Length variation" << endl;
//            cout << "1: " << v_lengths[0]  << "2: " << v_lengths[1] << "3: " << v_lengths[2] <<endl;
//            // Sensor data
//            misensor.GetPitchRollYaw(pitch,roll,yaw);
//            data <<ang[0] << " , " <<ang[1]<< " , "  << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
//            cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << endl; //CR
//            cout << endl;
//        }

//        //ang[1] = ang[1] + 5; //BETA
//        m1.SetPosition(0);
//        m2.SetPosition(0);
//        m3.SetPosition(0);

//        for (double t=0;t<interval; t+=dts)
//        {

//            misensor.GetPitchRollYaw(pitch,roll,yaw);
//            data <<ang[0] << " , " <<ang[1]<< " , " << pitch << " , " << yaw <<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
//            cout<< "Vuelta"<<endl;
//            cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw<< endl; //CR
//            cout << endl;

//            Ts.WaitSamplingTime();
//        }
////        cout <<"Posicion 0"<< endl;
////        cout << "Calibrating IMU..." << endl;
////        misensor.calibrate();
////        cout << "Calibration done" << endl;
////        sleep(2);
//    }

//    m3.SetPosition(0);
//    m2.SetPosition(0);
//    m1.SetPosition(0);

////    m3.SetPosition(0);
////    m2.SetPosition(0);
////    m1.SetPosition(0);
//    //sleep(4);

