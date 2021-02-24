#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"

int main ()
{

    ofstream data("/home/humasoft/Softartm/Motor_Softarm/graficas_demos/PRE_demo360_30.csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(6,6);

    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(6,6);

    SocketCanPort pm3("can1");
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(6,6);

    TableArmKinematics a("../Motor_Softarm/Tabla170.csv");
    vector<double> lengths(3);

    long orient=0;
    long incli=30;
    double lg0=0.204;
    double radio=0.0093;
    double dts=0.01;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    for (int i=0;i<=359;i++)

    {

//        if (orient >= 85 && orient <= 95)
//        {
//            orient =96;
//        }
//        if (orient >= 205 && orient <= 215)
//        {
//            orient =216;
//        }
//        if (orient >= 325 && orient <= 335)
//        {
//            orient =336;
//        }

    a.GetIK(incli,orient,lengths);
    double posan1, posan2, posan3;
    posan1=(lg0-lengths[0])/radio;
    posan2=(lg0-lengths[1])/radio;
    posan3=(lg0-lengths[2])/radio;


// Ponerlo en negativo (el motor va al reves)
      m3.SetPosition(-posan3);
      m2.SetPosition(-posan2);
      m1.SetPosition(-posan1);

    for (double t=0;t<10*dts;t+=dts)
    {
        cout << "Orientacion " << orient  << "  Inclinacion "  << incli <<endl;
        cout << "Pos angular1 " << posan1  << ", Pos angular2 " << posan2 << ", Pos angular3 " << posan3 <<endl;
        cout <<"t: "<<t<< " encoder1     " << -m1.GetPosition() << ", encoder2  " << -m2.GetPosition() << ", encoder3  " << -m3.GetPosition()<< endl;
        data <<incli<<" , "<<orient<<" , "<< -posan1  <<" , "<< m1.GetPosition()<<" , " << -posan2  <<" , "<<m2.GetPosition()<<" , "<< -posan3  <<" , "<<m3.GetPosition()<< endl;
        cout << endl;
        Ts.WaitSamplingTime();
    }
     orient=orient+1;
   }

    m3.SetPosition(0);
    m2.SetPosition(0);
    m1.SetPosition(0);
    float interval=2; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        data <<0<<" , "<<0<<" , "<< 0  <<" , "<< m1.GetPosition()<<" , " << 0  <<" , "<<m2.GetPosition()<<" , "<< 0  <<" , "<<m3.GetPosition()<< endl;
        cout << endl;
        Ts.WaitSamplingTime();
    }

}
