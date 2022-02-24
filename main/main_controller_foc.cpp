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
    ang[1] =40; //BETA

    double vel=5;
    string masa ="0"; // "0" "200" "500"
    bool windUp=false;

    string swind="";
    if (windUp==true) swind="W";

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Control/Control_Fraccionario/Masa_"+masa+"/Control_FOC"+swind+"_Vel"+to_string(int(vel))+"_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
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
    IPlot probe4(dts,"Plot m2");

    //FRACIONARIO
    //FPDBlock conP(0.2039,1.8421,-1,dts); //(kp,kd,exp,dts) ??


    //FPDBlock conP(0.3092,0.6175,-1,dts); //(kp,kd,exp,dts) wps=0.5 pm=90
    //FPDBlock conY(0.2931,0.4547,-1,dts); //(kp,kd,exp,dts) wps=0.5 pm=90

    // NEW
    FPDBlock conP(0.0123,1.2712,-0.99,dts); //(kp,kd,exp,dts) wps=1 pm=65
    FPDBlock conY(0.0905,0.9614,-0.96,dts);

//    FPDBlock resetP(conP); //Used for control reset
//    FPDBlock resetY(conY); //Used for control reset

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

        //controller computes control signal FPD
        cs[0] = ierror[0] > conP;
        cs[1] = ierror[1] > conY;

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
    //conP = FPDBlock(resetP); //Reset?
    //conY = FPDBlock(resetY); //Reset?

    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    //probe4.Plot();


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

}
