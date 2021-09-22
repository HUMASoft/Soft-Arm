#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
    string motor="m1";
    int sentido=-1; // Neg Acortar Posi Alargar


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

//    m1.Reset();
//    m1.SwitchOn();

    // motors must be turned ON


//    double pos;

//    double vel;

    ////VELOCIDAD negativo es acortar


    if (motor=="m1"){
        m1.Setup_Velocity_Mode();
        cout<< "Moving m1";
        m1.SetVelocity(sentido*1);
    }

    if (motor=="m2"){
        m2.Setup_Velocity_Mode();
        cout<< "Moving m2";
        m2.SetVelocity(sentido*1);
    }
    if (motor=="m3"){
        m3.Setup_Velocity_Mode();
        cout<< "Moving m3";
        m3.SetVelocity(sentido*1);
    }


    cout << "Enter to stop." <<endl;

//    // position  [rads]



    getchar();

    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);
    ////POSICION

//    cout << "Back 2 zero" << endl;
//    m1.SetupPositionMode();

//    m1.SetPosition(0);

//    m2.SetupPositionMode();

//    m2.SetPosition(0);

//    m3.SetupPositionMode();

//    m3.SetPosition(0);
//    cout << "Enter to stop." <<endl;

    // position  [rads]






}
