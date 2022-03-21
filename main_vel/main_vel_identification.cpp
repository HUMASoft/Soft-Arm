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
    double interval=8;
    double amp=2;
    bool test_pitch=true;
    bool test_yaw=false;
    string test_type="";


    if (test_pitch && test_yaw){
        test_type="PY";
    }else if (test_pitch) {
        test_type="P";
    }else if (test_yaw) {
        test_type="Y";
    }



    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Vel/Identification/ID_sin_test"+test_type+to_string(int(amp))+"_interval"+to_string(int(interval))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--
    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.Setup_Velocity_Mode(5,0);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.Setup_Velocity_Mode(5,0);

    SocketCanPort pm3(can);
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.Setup_Velocity_Mode(5,0);


    //Mechanical characteristics
    // double radio=0.0093;
    vector<double> v_lengths(3);
    vector<double> ang(2);
    //double posan1, posan2, posan3;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMU3DMGX510 misensor("/dev/ttyUSB0",freq);

    double pitch,roll, yaw;




    // TIME
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //PLOT
    IPlot probe(dts,"Plot Pitch");
    IPlot probe1(dts,"Plot Yaw");
    IPlot probe2(dts,"Plot V1");
    IPlot probe3(dts,"Plot V2");
    IPlot probe4(dts,"Plot V3");


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<5;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;

    // Start test
    ang[0] = 0; //ALPHA
    ang[1] = 0; //BETA
    double Next=0;
    double N_interval=5;
    double i_int=interval/N_interval;
    i_int=4;


    ang[0]=0;
    // CHANGING ALPHA AND BETA
    for (double t=0;t<interval;t+=dts)
    {

        ang[0]=amp*sin(t);
        /*
        //SIN
        if (test_pitch){
            ang[0]=sin(t+M_PI_2);
        }
        if (test_yaw){
            ang[1]=sin(t+M_PI_2);
        }
        */

        //SQUARE

        /*
         *         if (t>3){
            ang[0]=-3;
        }
        if (t>6){
            ang[0]=0;
        }

        if (t>Next){
            ang[0]=amp;
            Next=Next+i_int;
            amp=amp*-1;
        }
        */

        //RAMPA





        cout << "Time " << t<< endl;

        cout<< "Vamos a ver "<< Next<< " y "<< ang[0] << endl;

        //ang[0]=0;
        //cout << "Alpha:  " << ang[0] << ", Beta:  " << ang[1] << endl;
        //ang[0]=0;
        v_lengths[0]=( ang[0] / 1.5);
        v_lengths[1]=( (ang[1] / 1.732) - (ang[0] / 3) );
        v_lengths[2]=( (ang[0] / -3) - (ang[1] / 1.732) );

        m1.SetVelocity(v_lengths[0]);
        m2.SetVelocity(v_lengths[1]);
        m3.SetVelocity(v_lengths[2]);

        misensor.GetPitchRollYaw(pitch,roll,yaw);

        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);
        probe2.pushBack(m2.GetVelocity());
        probe3.pushBack(v_lengths[1]);
        probe4.pushBack(m2.GetPosition());

        data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " << pitch*180/M_PI << " Yaw: " << yaw*180/M_PI<< endl; //CR
        //cout << endl;
        Ts.WaitSamplingTime();
    }

    probe.Plot();
    //probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    probe4.Plot();
    cout<<"Back to zero"<<endl;


    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);
    m1.SetupPositionMode(3,3);
    m2.SetupPositionMode(3,3);
    m3.SetupPositionMode(3,3);
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

    for (double t=0;t<3; t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        data <<0 << " , " <<0<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR

        Ts.WaitSamplingTime();
    }
    misensor.Reset();
    //sleep(2);
    cout<<"end"<<endl;

}




