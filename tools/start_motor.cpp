#include <iostream>
#include "Cia402device.h"
//#include "CiA301CommPort.h"
#include "SocketCanPort.h"

int main ()
{
  string can = "can1";

  //m1 setup
  SocketCanPort pm31(can);
  CiA402SetupData sd31(2048,157,0.001, 1.25, 20 );
  CiA402Device m1 (31, &pm31, &sd31);
  m1.Reset();
  m1.SwitchOn();
  //m1.SetupPositionMode(10,10);


  //m2
  SocketCanPort pm2(can);
  CiA402SetupData sd32(2048,157,0.001, 1.25, 20 );
  CiA402Device m2 (32, &pm2, &sd32);
  m2.Reset();
  m2.SwitchOn();
  //m2.SetupPositionMode(10,10);

  //m3
  SocketCanPort pm3(can);
  CiA402SetupData sd33(2048,157,0.001, 1.25, 20 );
  CiA402Device m3 (33, &pm3, &sd33);
  m3.Reset();
  m3.SwitchOn();
  //m3.SetupPositionMode(10,10);


  return 0;

  }

