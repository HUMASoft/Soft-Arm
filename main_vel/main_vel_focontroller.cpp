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
    ang[0] =0; //ALPHA
    ang[1] =0; //BETA
    string masa="0";
    // 1p5_80
    // 5_60

    // NO GRABA

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Vel/Control/FOC/2_60/Control_Masa_"+masa+"_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--

    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.Setup_Velocity_Mode(5,0);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.Setup_Velocity_Mode(5,0);

    SocketCanPort pm3(can);
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.Setup_Velocity_Mode(5,0);

    vector<double> v_lengths(3);

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

//    FPDBlock conP(0.148,0.0871,-0.29,dts); //FOC Pitch Band 1.5 PM 80
//    FPDBlock conY(-0.1318,-0.0554,-0.35,dts); //FOC YAW Band 1.5 PM 80
    FPDBlock conP(0.0601,0.3172,-0.39,dts); //FOC Pitch Band 2 PM 60
    FPDBlock conY(-0.0581,-0.2416,-0.4,dts); //FOC YAW Band 2 PM 60

    //FPDBlock conP(0.3168,0.7401,-0.52,dts); //FOC Pitch Jorge Band 5 PM 60
    //FPDBlock conY(-0.3083,-0.9967,-0.46,dts); //FOC YAW Jorge Band 5 PM 60
//    double num[5]={0,0.085772060066429,-0.247871985677167,0.238508579019849,-0.07640859297221};
    vector<double>  num(5);
    vector<double>  den(5);

    num={-0.089862194815923,0.281419410260342,-0.293360525912833,0.101803394370491,0};
    den={0.060438759633418,-1.1333102350899,3.085117496232294,-3.012245968645809,1};

    SystemBlock fPDp(num,den);

    num={0.07640859291221,-0.238508579019849,0.247871985677167,-0.085772060066429,0};

    den={0.041725532270869,-1.078494548630914,3.031640050655836,-2.994870988390003,1};
    SystemBlock fPDy(num,den);

    fPDp.PrintZTransferFunction(dts);
    fPDy.PrintZTransferFunction(dts);

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<5;t+=dts)
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

        ierror[0] = ang[0] - pitch*180/M_PI;
        ierror[1] = ang[1] - yaw*180/M_PI;

        //PLOT DE DATOS
        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);

//        cs[0] = ierror[0] > conP;
//        cs[1] = ierror[1] > conY;

        cs[0] = ierror[0] > fPDp;
        cs[1] = ierror[1] > fPDy;

        probe2.pushBack(cs[0]);
        probe3.pushBack(cs[1]);
        probe4.pushBack(m2.GetVelocity());


        //SIN Control 0
        //cs[0]=0;
        //cs[1]=0;


        if (!isnormal(cs[0])) cs[0] = 0;

        if (!isnormal(cs[1])) cs[1] = 0;


        v_lengths[0]=( cs[0] / 1.5);
        v_lengths[1]=( (cs[1] / 1.732) - (cs[0] / 3) );
        v_lengths[2]=( (cs[0] / -3) - (cs[1] / 1.732) );

        m1.SetVelocity(v_lengths[0]);
        m2.SetVelocity(v_lengths[1]);
        m3.SetVelocity(v_lengths[2]);

        data <<ang[0] << " , " <<ang[1]<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetPosition() <<" , "<<  m2.GetPosition() <<" , "<<  m3.GetPosition() <<" , "<<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
        Ts.WaitSamplingTime();
    }

    cout <<"Done" << endl;
    //conP = FPDBlock(resetP); //Reset?
    //conY = FPDBlock(resetY); //Reset?

    probe.Plot();
    probe1.Plot();
    //probe2.Plot();
    //probe3.Plot();
    //probe4.Plot();
    cout<<"Back to zero"<<endl;


    m1.SetVelocity(0);
    m2.SetVelocity(0);
    m3.SetVelocity(0);
    m1.SetupPositionMode(3,3);
    m2.SetupPositionMode(3,3);
    m3.SetupPositionMode(3,3);
    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

    for (double t=0;t<3; t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        data <<0 << " , " <<0<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetPosition() <<" , "<<  m2.GetPosition() <<" , "<<  m3.GetPosition() <<" , "<<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR

        Ts.WaitSamplingTime();
    }
    misensor.Reset();
    //sleep(2);
    cout<<"end"<<endl;
}
