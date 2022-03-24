#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
    string motor="m1";
    int velo=1;
    int sentido=-2; // Neg Acortar Posi Alargar


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


    ////VELOCIDAD negativo es acortar


    if (motor=="m1"){
        m1.Setup_Velocity_Mode();
        cout<< "Moving m1 "<<endl;

        m1.SetVelocity(sentido*velo);
    }

    if (motor=="m2"){
        m2.Setup_Velocity_Mode();
        cout<< "Moving m2 "<<endl;

        m2.SetVelocity(sentido*velo);
    }
    if (motor=="m3"){
        m3.Setup_Velocity_Mode();
        cout<< "Moving m3 "<<endl;

        m3.SetVelocity(sentido*velo);
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
