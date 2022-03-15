#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"

//
#include "imu3dmgx510.h"

int main ()
{
    vector<double> ang(2);
    ang[0] =40; //ALPHA
    ang[1] =0; //BETA

    double vel=3;
    string masa ="0"; // "200" "500"
    bool windUp=false;

    string scontrol= "PID";
    string swind="";
    if (windUp==true) swind="W";

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Control/Control_Robusto/Masa_"+masa+"/Control_RobPID"+swind+"_Vel"+to_string(int(vel))+"_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--

    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(vel,vel);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);
    m2.SetupPositionMode(vel,vel);

    SocketCanPort pm3(can);
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(vel,vel);

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
    IPlot probe(dts,"Plot Pitch ");
    IPlot probe1(dts,"Plot Yaw");
    IPlot probe2(dts,"Plot cs1");
    IPlot probe3(dts,"Plot cs2");
    //IPlot probe4(dts,"Plot m2");

    // CONTROLLER
    //PID ROBUSTO
    PIDBlock conPPID(0.3592,0.0783,0.4317,dts); //PI Pitch
    PIDBlock conYPID(0.3935,0.1353,0.2583,dts); //PI YAW

//    PIDBlock conPPID(0.972,1.51,0.124,dts); //PI Pitch  wsp=1.5 pm=100
//    PIDBlock conYPID(0.87,1.13,0.177,dts); //PI YAW


//    PIDBlock conPPID(0.972,1.51,0.124,dts); //PI Pitch  wsp=1.5 pm=100 NOP
//    PIDBlock conYPID(0.87,1.13,0.177,dts); //PI YAW

//    PIDBlock conPPID(0.815,1.79,0.148,dts); //PI Pitch  wsp=1.5 pm=90 NOP
  //  PIDBlock conYPID(0.772,1.33,0.172,dts); //PI YAW

    //PIDBlock conPPID(0.609,3.74,0.386,dts); //PI Pitch  wsp=2 pm=65 NOP
    //PIDBlock conYPID(0.697,2.61,0.281,dts); //PI YAW

//    PIDBlock conPPID(0.0325,12.3,11,dts); //PI Pitch  wsp=1 pm=65 NOP
//    PIDBlock conYPID(0.152,2.32,1.36,dts); //PI YAW

    //PIDBlock conPPID(0.6337,2.1946,0.2412,dts); //PI Pitch  wsp=1.5 pm 80
    //PIDBlock conYPID(0.651,1.5838,0.2012,dts); //PI YAW


    if(windUp==true){
        conPPID.AntiWindup(vel,vel);
        conYPID.AntiWindup(vel,vel);
     }


    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;
    cout<<"Moving to Pitch: "+to_string(int(ang[0]))+ " and Yaw: "+to_string(int(ang[1]))<<endl;

    double interval=10; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;

        ierror[0] = ang[0] - pitch*180/M_PI;
        ierror[1] = ang[1] - yaw*180/M_PI;

        //PLOT DE DATOS
        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);

        // SEÑAL CONTROL PID
        cs[0] = ierror[0] > conPPID;
        cs[1] = ierror[1] > conYPID;

        //PLOT DE Control
        probe2.pushBack(cs[0]);
        probe3.pushBack(cs[1]);

        //SIN Control 0
        //cs[0]=0;
        //cs[1]=0;

        if (!isnormal(cs[0])) cs[0] = 0;
        if (!isnormal(cs[1])) cs[1] = 0;

        // Calculo angulos motores
        v_lengths[0]=0.001*( cs[0] / 1.5);
        v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
        v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

        posan1=(v_lengths[0])/radio;
        posan2=(v_lengths[1])/radio;
        posan3=(v_lengths[2])/radio;

        // Enviando posicion motores
        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);

        //Guardando datos
        data <<ang[0] << " , " <<ang[1]<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
        Ts.WaitSamplingTime();
    }
    cout <<"Done" << endl;

    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    //probe4.Plot();


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

}