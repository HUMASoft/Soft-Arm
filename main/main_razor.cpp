#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"

//
#include "IMURazor9DOF.h"


int main ()
{

    vector<double> ang(2);
    ang[0] = 20; //ALPHA
    ang[1] = 0; //BETA


    ofstream data("/home/humasoft/code/Soft-Arm/graphs/TestRazor_Control_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
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

    double radio=0.0093;
    vector<double> v_lengths(3);
    double posan1, posan2, posan3;

    // SENSOR
    double freq=50; //sensor use values: 50,100,500...
    IMURazor9DOF misensor("/dev/ttyUSB0");

    double pitch,roll, yaw;
    double dts=1/freq;
    SamplingTime Ts;
    Ts.SetSamplingTime(dts);

    //plot
    IPlot probe(dts);
    IPlot probe2(dts);

    // CONTROLLER
    //FPDBlock conP(0.4506,0.5478,-1.11,dts); //(kp,kd,exp,dts) 0.0214437 90 0.5
    //FPDBlock conP(0.7996,0.8271,-1.17,dts); //80 0.8
    //FPDBlock conP(0.5811,0.5178,-0.97,dts); //(kp,kd,exp,dts) 0.0214437 100 0.5
    PIDBlock conPPID(0,4,0,dts);

    //FPDBlock conY(0.6426,0.576,-1.11,dts); //(kp,kd,exp,dts) 0.0214437 90 0.5
    //FPDBlock conY(1.232,0.8582,-1.24,dts); //80 0.8
    //FPDBlock conY(0.8508,0.4978,-1.02,dts); //(kp,kd,exp,dts) 0.0214437 100 0.5
    PIDBlock conYPID(2.8,4,1,dts);

    //FPDBlock resetP(conP); //Used for control reset
    //FPDBlock resetY(conY); //Used for control reset

//    SystemBlock imuFilter(0.09516,0,- 0.9048,1); //w=5

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    //misensor.set_streamon();

    for (double t=0;t<2;t+=dts)
    {
        //misensor.GetPitchRollYaw(pitch,roll,yaw);

        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
        misensor.GetYawPitchRoll(dts, yaw, pitch, roll );
        cout<<-roll<<endl;
    }
    cout<<"Calibrado"<<endl;
    double offset_yaw = yaw;
    cout<<"Moving to Pitch: "+to_string(int(ang[0]))+ " and Yaw: "+to_string(int(ang[1]))<<endl;

    double interval=20; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        misensor.GetYawPitchRoll(dts, yaw, pitch, roll );

        yaw=yaw-offset_yaw;
        //yaw=-yaw;
        pitch= -roll ;


        cout << "-> pitch: " << pitch << ", yaw: " << yaw << endl;
        ierror[0] = ang[0] - pitch;
        ierror[1] = ang[1] - yaw;

        //ierror= ierror*M_PI/180; //degrees to rad

        //PLOT DE DATOS
        probe.pushBack(pitch);
        probe2.pushBack(yaw);

        //controller computes control signal FPD
        //cs[0] = ierror[0] > conP;
        //cs[1] = ierror[1] > conY;

        //controller computes control signal PID

        cs[0] = ierror[0] > conPPID;
//        cs[1] = ierror[1] > conYPID;
//        cout << endl;

        cs[1]=0;

        if (!isnormal(cs[0])) cs[0] = 0;

        if (!isnormal(cs[1])) cs[1] = 0;

        v_lengths[0]=0.001*( cs[0] / 1.5);
        //v_lengths[1]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );
        //v_lengths[2]=0.001*( (cs[0] / -3) - (cs[1] / 1.732) );

        // INVERTIDO
        v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) ); //Antiguo tendon 3
        v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) ); //Antiguo tendon 2

        posan1=(v_lengths[0])/radio;
        posan2=(v_lengths[1])/radio;
        posan3=(v_lengths[2])/radio;

        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);

        data <<ang[0] << " , " <<ang[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() << " , " << cs[0] << " , " <<cs[1] << endl; //CR
        //cout << endl;
        Ts.WaitSamplingTime();
    }
    cout <<"Done" << endl;
    //conP = FPDBlock(resetP); //Reset?
    //conY = FPDBlock(resetY); //Reset?

    probe.Plot();
    probe2.Plot();

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
    sleep(4);
}
