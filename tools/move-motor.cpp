#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>


int main ()
{
    //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20); //(2*2048,3.7,0.001, 1.1)(2048,24,0.001, 0.144)
    CiA402Device m (32, &pm1, &sd1);


    // motors must be turned ON


//    double pos;
//    double vel;

//    m.Reset();
//    m.SwitchOn();
    m.Setup_Velocity_Mode();
//    m.SetupPositionMode();

    cout << "Enter to stop." <<endl;

    // position  [rads]
    m.SetVelocity(1);
//    m.SetPosition(2);

    getchar();

    m.SetVelocity(0);


}
