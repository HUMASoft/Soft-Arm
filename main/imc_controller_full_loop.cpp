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
    vector<double> ang(2); //REPETIR 40 0 masa
    ang[0] =40; //ALPHA
    ang[1] =0; //BETA
    //ang[1]=ang[1]/2;
    string masa="500g";

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Vel/Control/Control_IMC/COntrol_IMC_Masa_"+masa+"_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
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




//    1.5 A

//    vector<double>  numa(4);
//    vector<double>  dena(4);
//    numa={0.00175583939884698, 0.00772975202407191, -0.0207316410185999, 0.0112460500060088  };
//    dena={ -0.948844907999521,  2.89768075854112, -2.9488358505416, 1};
//    SystemBlock imca(numa,dena);


////    1.5 B
//    vector<double>  numb(3);
//    vector<double>  denb(3);
//    numb={-0.00184949121128251, +0.0140070924627311,   -0.0121588473479795};
//    denb={ 0.949013494760995,  -1.94901349476099, 1};
//    SystemBlock imcb(numb,denb);


    //    Planta A
    vector<double>  numGa={ 0.0380092407788771,   0.0562252261565164, 0};
    vector<double>  denGa={ 0.305203149549575,  -1.30507973253859, 1};
    SystemBlock Ga(numGa,denGa);
    //    Planta B
    vector<double>  numGb={-0.0425880237292791, -0.0568301994450454, 0};
    vector<double>  denGb={ 0.418817137640668, -1.41874689462195, 1};
    SystemBlock Gb(numGb,denGb);

//    //    1.5 A
//    vector<double>  numa={-0.00176223096206252,  -0.00948150956370442,   0.0112460500060088};
//    vector<double>  dena={ 0.949013494760995,  -1.94725010219667, 1};
//    SystemBlock Ca(numa,dena);
//    //    1.5 B
//    vector<double>  numb={ -0.00184178731343784,   0.0139993887528411,  -0.0121588473479795};
//    vector<double>  denb={ 0.949013494760995,  -1.94725010219667,1};
//    SystemBlock Cb(numb,denb);


    //    1.5 A
    vector<double>  numa={-0.00176223096206252,  -0.00948150956370442,   0.0112460500060088};
    vector<double>  dena={ 0.949013494760995,  -1.94725010219667, 1};
    SystemBlock Ca(numa,dena);
    //    1.5 B
    vector<double>  numb={ -0.00184178731343784,   0.0139993887528411,  -0.0121588473479795};
    vector<double>  denb={ 0.949013494760995,  -1.94725010219667,1};
    SystemBlock Cb(numb,denb);

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<5;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;
    cout<<"Moving to Pitch: "+to_string(int(ang[0]))+ " and Yaw: "+to_string(int(ang[1]))<<endl;

    double interval=10; //in seconds
    double ya=0,yb=0;
    for (double t=0;t<interval; t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;

//        ang[0]=10*sin(t);
//        ang[1]=10*cos(t);



        ierror[0] = ang[0] - (pitch*180/M_PI-ya);
        ierror[1] = ang[1] - (yaw*180/M_PI-yb);

        //ierror= ierror*M_PI/180; //degrees to rad
        probe4.pushBack(ierror[0]);
        //PLOT DE DATOS
        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);

        // SEÑAL CONTROL PID

        cs[0] = ierror[0] > Ca;
        cs[1] = ierror[1] > Cb;
        probe2.pushBack(cs[0]);
        probe3.pushBack(cs[1]);

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

        ya = cs[0] > Ga;
        yb = cs[1] > Gb;


        //data <<ang[0] << " , " <<ang[1]<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetPosition() <<" , "<<  m2.GetPosition() <<" , "<<  m3.GetPosition() <<" , "<<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
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
        //data <<0 << " , " <<0<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  v_lengths[0] <<" , " <<v_lengths[1] <<" , " <<v_lengths[2]<<" , " <<  m1.GetPosition() <<" , "<<  m2.GetPosition() <<" , "<<  m3.GetPosition() <<" , "<<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR

        Ts.WaitSamplingTime();
    }
    misensor.Reset();
    //sleep(2);
    cout<<"end"<<endl;
}
