#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"
#include "imu3dmgx510.h"
#include <string>

int main ()
{

    vector<double> ang(2);
    ang[0] = 0; //ALPHA
    ang[1] = 0; //BETA

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



    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    ifstream csv;
    csv.open("/home/humasoft/code/Soft-Arm/Tables/results_javi.csv");
    //csv.seekg(0);

    string line;
    string word="0.1";

    double i1,i2;
    double mot1,mot2,mot3;

    for (double t=0;t<8;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;


    for (int i=1;i<=20;i++){
        getline(csv,line);
        //cout << line << endl;


        istringstream sstream(line);



        getline(sstream, word, ',' );
        mot1=stod(word);
        getline(sstream, word, ',' );
        mot2=stod(word);
        getline(sstream, word, ',' );
        mot3=stod(word);

        getline(sstream, word, ',' );
        i1=stod(word);
        getline(sstream, word, ',' );
        i2=stod(word);


//        cout << i1<<" " << i2<<" "  << mot1<< " "  <<mot2<< " " << mot3 <<" " <<line << endl;

        ofstream data("/home/humasoft/code/Soft-Arm/graphs/Javi_Redes/Demo_inv_P"+to_string(int(i1))+"_Y"+to_string(int(i2))+".csv",std::ofstream::out); // /home/humasoft/code/graficas





        double interval=6; //in seconds

        for (double t=0;t<interval; t+=dts)
        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);

            probe.pushBack(pitch*180/M_PI);
            probe1.pushBack(yaw*180/M_PI);
            //            probe2.pushBack(cs[0]);
            //            probe3.pushBack(cs[1]);
            probe2.pushBack(m2.GetPosition());
            probe3.pushBack(m2.GetVelocity());


            posan1=mot1*M_PI/180;
            posan2=mot2*M_PI/180;
            posan3=mot3*M_PI/180;


            //            probe4.pushBack(posan2);

            m1.SetPosition(posan1);
            m2.SetPosition(posan2);
            m3.SetPosition(posan3);

            data << i1 << " , " <<i2<< " , " << mot1<< " , " << mot2<< " , " << mot3<<" , " <<  roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << endl; //CR
            Ts.WaitSamplingTime();
        }
        interval=3;
        for (double t=0;t<interval; t+=dts)
        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);

            probe.pushBack(pitch*180/M_PI);
            probe1.pushBack(yaw*180/M_PI);
            //            probe2.pushBack(0);
            //            probe3.pushBack(0);
            probe2.pushBack(m2.GetPosition());
            probe3.pushBack(m3.GetPosition());

            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);
            Ts.WaitSamplingTime();
        }

    }

    cout <<"Done" << endl;
    probe.Plot();
    probe1.Plot();
    //probe2.Plot();
    //probe3.Plot();
    //    probe4.Plot();

}
