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


    ofstream data("/home/humasoft/Soft-Arm/graphs/PRE_demodegs5.csv",std::ofstream::out); // /home/humasoft/code/graficas
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

    TableArmKinematics a("Tabla170.csv");
    vector<double> lengths(3);

    double incli=30.0; //inclination tendon length
    double orient=0*M_PI/3; //target orientation
    double lg0=0.2;
    double radio=0.0093;
    double dts=0.01;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);


    for (long stops = 4; stops > 0 ; stops--)
    {

        double interval=2; //in seconds
        for (double t=0;t<interval; t+=dts)
        {

            a.GetIK(incli,orient,lengths);
            cout << "Orientacion " << orient  << "  Inclinacion "  << incli <<endl;
            cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
            double posan1, posan2, posan3;
            posan1=(lg0-lengths[0])/radio;
            posan2=(lg0-lengths[1])/radio;
            posan3=(lg0-lengths[2])/radio;
            cout << "Pos angular1 " << posan1  << ", Pos angular2 " << posan2 << ", Pos angular3 " << posan3 <<endl;


            // motors must be turned ON

            // Ponerlo en negativo (el motor va al reves)
            m3.SetPosition(posan3);
            m2.SetPosition(posan2);
            m1.SetPosition(posan1);

            cout << "encoder1     " << m1.GetPosition() << ", encoder2  " << m2.GetPosition() << ", encoder3  " << m3.GetPosition()<< endl;
            data <<incli<<" , "<<orient<<" , "<< posan1  <<" , "<< m1.GetPosition()<<" , " << posan2  <<" , "<<m2.GetPosition()<<" , "<< posan3  <<" , "<<m3.GetPosition()<< endl;
            cout << endl;
            Ts.WaitSamplingTime();
        }


        // sleep(1);
        orient=orient+90;
        m1.SetPosition(0);
        m2.SetPosition(0);
        m3.SetPosition(0);
        interval=2; //in seconds
        for (double t=0;t<interval; t+=dts)
        {
            cout << "encoder1     " << -m1.GetPosition() << ", encoder2  " << -m2.GetPosition() << ", encoder3  " << -m3.GetPosition()<< endl;
            data <<0<<" , "<<0<<" , "<< 0  <<" , "<< m1.GetPosition()<<" , " << 0  <<" , "<<m2.GetPosition()<<" , "<< 0  <<" , "<<m3.GetPosition()<< endl;
            cout << endl;
            Ts.WaitSamplingTime();
        }
        sleep(4);

    }

    m3.SetPosition(0);
    m2.SetPosition(0);
    m1.SetPosition(0);
    //sleep(4);
}
