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
    //ang[1]=ang[1]/2;
    double vel=3;

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Control/Control_PIDW/Control_FOC_Vel5_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--

    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(vel,vel);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
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
    IMU3DMGX510 misensor("/dev/ttyUSB1",freq);

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




//    vector<double> num{0.310518571010877, - 1.269831550003351, 1.94665647051899, - 1.32589305870901, 0.338549562526197};
//      vector<double> den{0.992340708803225, - 3.977011501641942, 5.97700087383432, - 3.992330080995814, 1 };
vector<double> num{-0.370501300209724,1.88102356335366,-3.81947225108335,3.87728952347584,-1.96775004692310,0.399410511386824,0};
vector<double> den{0,-0.988392893410266,4.95354383990612,-9.93027414213765,9.95348833819555,-4.98836514255375,1};
    //5,6
//vector<double> num{};
//vector<double> den{

//4,5
//[0,0.316350842472026,-1.23678867985548,1.81250326122132,-1.18004348566005,0.287978061928761]
//[1,-3.99336500780094,5.98010293749690,-3.98011084973228,0.993372920036334,5.33919633271061e-24]

    //    vector<double> num{0.000438*180/M_PI,0.0004682*180/M_PI,0};
//    vector<double> den{0.8187,-1.817,1};
//    vector<double> num{0.675716,-0.581882,0};
//    vector<double> den{0.0263503,-0.905029,1};
//    SystemBlock pitchModel(num,den);
//    double pitchPred;

    //identification
//    OnlineSystemIdentification pitchOnlineModel(1,2);



    // CONTROLLER
    //PIDSIMPLE
//    PIDBlock conPPID(0.2039,1.8421,0,dts); //PI Pitch
    SystemBlock fPD(num,den);
//    PIDBlock conPPID(0.3030,1.8394,0.0125,dts); //PID Pitch
    PIDBlock conYPID(0.23762,0.9464,0,dts); //PI YAW
//    PIDBlock conYPID(0.2962,1.2143,0.0181,dts); //PID YAW

//    conPPID.AntiWindup(3,3);
//    conYPID.AntiWindup(3,3);

    //ROBUSTO


    //FRACIONARIO
//    FPDBlock conP(0,3,-0.7,dts); //(kp,kd,exp,dts)
//    FPDBlock conY(0.8508,0.4978,-1.02,dts); //(kp,kd,exp,dts) 0.0214437 100 0.5

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

//        ang[0]=10*sin(t);
//        ang[1]=10*cos(t);

        ierror[0] = ang[0] - pitch*180/M_PI;
        ierror[1] = ang[1] - yaw*180/M_PI;

        //ierror= ierror*M_PI/180; //degrees to rad
        probe4.pushBack(ierror[0]);
        //PLOT DE DATOS
        probe.pushBack(pitch*180/M_PI);
        probe1.pushBack(yaw*180/M_PI);

        // SEÑAL CONTROL PID
//        cs[0] = ierror[0] > conPPID;
        cs[0] = ierror[0] > fPD;
        cs[1] = ierror[1] > conYPID;
        probe2.pushBack(cs[0]);
        probe3.pushBack(cs[1]);

//        controller computes control signal FPD
//        cs[0] = ierror[0] > conP;
//        cs[1] = ierror[1] > conY;



        //SIN Control 0
        //cs[0]=0;
        //cs[1]=0;

        if (!isnormal(cs[0])) cs[0] = 0;

        if (!isnormal(cs[1])) cs[1] = 0;


        v_lengths[0]=0.001*( cs[0] / 1.5);
        v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
        v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

        posan1=(v_lengths[0])/radio;
        posan2=(v_lengths[1])/radio;
        posan3=(v_lengths[2])/radio;

        //probe2.pushBack(m1.GetPosition());
        //probe3.pushBack(posan1);
        //probe4.pushBack(cs[0]);

        m1.SetPosition(posan1);
        m2.SetPosition(posan2);
        m3.SetPosition(posan3);
        data <<ang[0] << " , " <<ang[1]<< " , " <<cs[0] << " , " <<cs[1]<< " , " << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition()<<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps()  << endl; //CR
        cout << t <<endl;
        Ts.WaitSamplingTime();
    }
    cout <<"Done" << endl;
    //conP = FPDBlock(resetP); //Reset?
    //conY = FPDBlock(resetY); //Reset?

    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
    probe4.Plot();


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
//    sleep(4);
}
