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
    string masa ="500"; // "200" "500"
    bool control_pi=true;
    bool windUp=false;

    string scontrol= "PID";
    string swind="";
    if (control_pi==true) scontrol="PI";
    if (windUp==true) swind="W";

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/Control/Control_Scheduling/Masa_"+masa+"/Control_Sch_"+scontrol+swind+"_Vel"+to_string(int(vel))+"_P"+to_string(int(ang[0]))+"_Y"+to_string(int(ang[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas
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
    //PID SIMPLE

    double kpa,kia,kda;
    double kpb,kib,kdb;

    // leer datos

    ifstream csv;
    csv.open("/home/humasoft/code/Soft-Arm/Tables/"+scontrol+"_scheduling.csv");
    csv.seekg(0);

    string line;
    cout << "PID busqueda"<< endl;
    int cnt=1;
    int cntF;
    string dato,val;

    for (double alp=-50 ; alp <= 50 ; alp= alp+10)
    {
        for (double bet=-50 ; bet <= 50 ; bet= bet+10)
        {
            getline(csv,line);

            if (alp==ang[0] && bet==ang[1])
            {
                istringstream ss(line);

                std::getline(ss, dato);

                istringstream ss2(dato);

                std::getline(ss2, val,',');
                kpa=stod(val);
                std::getline(ss2, val,',');
                kia=stod(val);
                std::getline(ss2, val,',');
                kda=stod(val);
                std::getline(ss2, val,',');
                kpb=stod(val);
                std::getline(ss2, val,',');
                kib=stod(val);
                std::getline(ss2, val,',');
                kdb=stod(val);

                //cout << "PID encontrado"<< endl;
                //cout << kpa<<" "<<kia<<" "<<kda<<" "<< endl;
                //cout << kpb<<" "<<kib<<" "<<kdb<<" "<< endl;
                //cntF=cnt;
                //cout << cntF<< endl;
                break;

            }
            //cnt=cnt+1;

        }
    }

    PIDBlock conPPID(kpa,kia,kda,dts); //PI Pitch
    PIDBlock conYPID(kpb,kib,kdb,dts); //PI YAW



    //
    if(windUp==true){
        conPPID.AntiWindup(vel,vel);
        conYPID.AntiWindup(vel,vel);
     }


    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();


    double interval=10;

    for (double t=0;t<interval;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;
    cout<<"Moving to Pitch: "+to_string(int(ang[0]))+ " and Yaw: "+to_string(int(ang[1]))<<endl;

     //in seconds
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

    //probe.Plot();
    //probe1.Plot();
    //probe2.Plot();
    //probe3.Plot();
    //probe4.Plot();


    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);

}
