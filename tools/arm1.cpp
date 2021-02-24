#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>


int main ()
{


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
    //    m3.Setup_Velocity_Mode();

    TableArmKinematics a("../Motor_Softarm/Tabla170.csv");
    vector<double> lengths(3);

    long orient=0;
    long incli=45;
    double lg0=0.2;
    double radio=0.009;

    a.GetIK(incli,orient,lengths);    //// PORQUE AQUI
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    double posan1, posan2, posan3;


//    posan1=(lg0-lengths[0])*180/(radio*M_PI);
//    posan2=(lg0-lengths[1])*180/(radio*M_PI);
//    posan3=(lg0-lengths[2])*180/(radio*M_PI);

    posan1=(lg0-lengths[0])/radio;
    posan2=(lg0-lengths[1])/radio;
    posan3=(lg0-lengths[2])/radio;

    cout << "Posicion angular 1 " << posan1  << ", Posicion angular 2 " << posan2 << ", Posicion angular 3 " << posan3 <<endl;


   for (int i=0;i<=10;i++) {

      cout<< "prueba #"  << i+1 <<endl;
      m3.SetPosition(-posan3);
      m2.SetPosition(-posan2);
      m1.SetPosition(-posan1);

    sleep(2);

    cout << "Pos final"<< endl;
    cout << m1.GetPosition() << endl;
    cout << m2.GetPosition() << endl;
    cout << m3.GetPosition() << endl;

    m3.SetPosition(0);
    m2.SetPosition(0);
    m1.SetPosition(0);

    sleep(1);
   }

}
