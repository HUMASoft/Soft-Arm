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

    vector<double> ang(2);
    ang[0] = 40; //ALPHA
    ang[1] = 0; //BETA



    cout<< "456"<<endl;

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Trayectoria_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
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

    //plot
    IPlot probe(dts,"Plot Pitch");
    IPlot probe1(dts,"Plot Yaw");
    IPlot probe2(dts,"Plot CPitch");
    IPlot probe3(dts,"Plot m2");
    IPlot probe4(dts,"Plot m3");

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL
    vector<double> valores(8);
    valores[0]=ang[0];
    valores[1]=0;
    valores[2]=0;
    valores[3]=-ang[0];
    valores[4]=0;
    valores[5]=ang[0];
    valores[6]=-ang[0];
    valores[7]=0;

//    //TEST
//    valores[0]=ang[0];
//    valores[4]=ang[1];


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;

     double interval=3; //in seconds
    for (long move = 0; move < 4 ; move++)
    {
        cs[0]=valores[move];
        cs[1]=valores[move+4];

        cout<<"Moving to Input Pitch: "+to_string(int(cs[0]))+ " and Yaw: "+to_string(int(cs[1]))<<endl;
        for (double t=0;t<interval; t+=dts)
        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);
            probe.pushBack(pitch*180/M_PI);
            probe1.pushBack(yaw*180/M_PI);
            if (!isnormal(cs[0])) cs[0] = 0;
            if (!isnormal(cs[1])) cs[1] = 0;

            v_lengths[0]=0.001*( cs[0] / 1.5);
            v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
            v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

            posan1=(v_lengths[0])/radio;
            posan2=(v_lengths[1])/radio;
            posan3=(v_lengths[2])/radio;

          probe2.pushBack(cs[0]);
//            probe3.pushBack(m2.GetPosition());
//            probe4.pushBack(posan2);

            m1.SetPosition(posan1);
            m2.SetPosition(posan2);
            m3.SetPosition(posan3);

            data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << " , " << cs[0] << " , " <<cs[1] << endl; //CR
            //cout << endl;
            Ts.WaitSamplingTime();
        }


      }
      interval=3;
      cs[0]=0;
      cs[1]=30;
      for (double t=0;t<interval; t+=dts)
      {
          misensor.GetPitchRollYaw(pitch,roll,yaw);
          probe.pushBack(pitch*180/M_PI);
          probe1.pushBack(yaw*180/M_PI);

          v_lengths[0]=0.001*( cs[0] / 1.5);
          v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
          v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

          posan1=(v_lengths[0])/radio;
          posan2=(v_lengths[1])/radio;
          posan3=(v_lengths[2])/radio;

          m1.SetPosition(posan1);
          m2.SetPosition(posan2);
          m3.SetPosition(posan3);

          probe2.pushBack(cs[0]);
          Ts.WaitSamplingTime();
      }
      interval=20;
      for (double t=0;t<interval; t+=dts)
      {
          cs[0]=sin(t/3)*30; //ang[0];
          cs[1]=cos(t/3)*30; //*ang[0];
          misensor.GetPitchRollYaw(pitch,roll,yaw);
          probe.pushBack(pitch*180/M_PI);
          probe1.pushBack(yaw*180/M_PI);
          if (!isnormal(cs[0])) cs[0] = 0;
          if (!isnormal(cs[1])) cs[1] = 0;

          v_lengths[0]=0.001*( cs[0] / 1.5);
          v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
          v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

          posan1=(v_lengths[0])/radio;
          posan2=(v_lengths[1])/radio;
          posan3=(v_lengths[2])/radio;

          probe2.pushBack(cs[0]);
//            probe3.pushBack(m2.GetPosition());
//            probe4.pushBack(posan2);

          m1.SetPosition(posan1);
          m2.SetPosition(posan2);
          m3.SetPosition(posan3);

          data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << " , " << cs[0] << " , " <<cs[1] << endl; //CR
          //cout << endl;
          Ts.WaitSamplingTime();
      }
      cout <<"Done" << endl;
      m1.SetPosition(0);
      m2.SetPosition(0);
      m3.SetPosition(0);

      probe.Plot();
      probe1.Plot();
      probe2.Plot();
//      probe3.Plot();
//    probe4.Plot();
}
