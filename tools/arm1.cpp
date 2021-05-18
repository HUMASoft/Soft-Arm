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


    TableArmKinematics a("Tabla170.csv");
    vector<double> lengths(3);

    double incli=50.0; //inclination tendon length
    double orient=0*M_PI/3; //target orientation
    double lg0=0.2;
    double radio=0.0093;
    double dts=0.01;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    a.GetIK(incli,orient,lengths);

    cout << "Orientacion " << orient  << "  Inclinacion "  << incli <<endl;
    cout << "l1 " << lengths[0]  << ", l2 " << lengths[1] << ", l3 " << lengths[2]<<endl;
    double posan1, posan2, posan3;
    posan1=(lg0-lengths[0])/radio;
    posan2=(lg0-lengths[1])/radio;
    posan3=(lg0-lengths[2])/radio;

    cout << "Pos angular1 " << posan1  << ", Pos angular2 " << posan2 << ", Pos angular3 " << posan3 <<endl;


    // motors must be turned ON

}
