#include <iostream>
#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"

int main ()
{

  //m1 setup

  string can = "can0";

  SocketCanPort pm31(can);
  CiA402SetupData sd31(2048,157,0.001, 1.25, 20 );
  CiA402Device m1 (31, &pm31, &sd31);

  m1.SwitchOff();
  //m1.SetupPositionMode(10,10);


  //m2
  SocketCanPort pm2(can);
  CiA402SetupData sd32(2048,157,0.001, 1.25, 20 );
  CiA402Device m2 (32, &pm2, &sd32);
  m2.SwitchOff();
  //m2.SetupPositionMode(10,10);

  //m3
  SocketCanPort pm3(can);
  CiA402SetupData sd33(2048,157,0.001, 1.25, 20 );
  CiA402Device m3 (33, &pm3, &sd33);
  m3.SwitchOff();
  //m3.SetupPositionMode(10,10);


  return 0;

  }

