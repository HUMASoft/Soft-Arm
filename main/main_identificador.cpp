//main/main_identificador.cpp

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
    bool d_random=1;
    vector<double> ang(2);
    ang[0] = 50; //ALPHA
    ang[1] =50; //BETA
    //ang[1]=ang[1]/2;

       //--Can port communications--
    SocketCanPort pm1("can1");
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(3,3);

    SocketCanPort pm2("can1");
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(3,3);

    SocketCanPort pm3("can1");
    CiA402SetupData sd3(2048,157,0.001, 1.25, 20 );
    CiA402Device m3 (33, &pm3, &sd3);
    m3.SetupPositionMode(3,3);

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


    //identification
    ulong numOrder=0,denOrder=2;
    ulong numOrder2=0,denOrder2=3;

    OnlineSystemIdentification modelP(numOrder, denOrder );
    OnlineSystemIdentification modelP2 (numOrder2, denOrder2 );

    OnlineSystemIdentification modelY(numOrder, denOrder );
    OnlineSystemIdentification modelY2 (numOrder2, denOrder2 );


    vector<double> numP(numOrder+1),denP(denOrder+1); //(order 0 also counts)
    SystemBlock sysP(numP,denP); //the resulting identification

    vector<double> numP2(numOrder2+1),denP2(denOrder2+1); //(order 0 also counts)
    SystemBlock sysP2(numP2,denP2); //the resulting identification

    vector<double> numY(numOrder+1),denY(denOrder+1); //(order 0 also counts)
    SystemBlock sysY(numY,denY); //the resulting identification

    vector<double> numY2(numOrder2+1),denY2(denOrder2+1); //(order 0 also counts)
    SystemBlock sysY2(numY2,denY2); //the resulting identification

    double gainP;
    double gainP2;
    double gainY;
    double gainY2;


    IPlot probe(dts,"Plot Pitch");
    IPlot probe2(dts,"Plot Input");

    vector<double> cs(2); //CONTROL SIGNAL


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
    }
    cout<<"Calibrado"<<endl;

    if(d_random==1){

        cs[0]=0;
        cs[1]=0;
        double tmax=5;
        double iderror=0;

        ofstream data("/home/humasoft/code/Soft-Arm/graphs/Identificacion/IndentificacionRand.csv",std::ofstream::out); // /home/humasoft/code/graficas

        for (double t=0; t<tmax; t+=dts)

        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);
            //cs[0]=1+0.0001*((rand() % 10 + 1)-5); //u_{i-1}
            cs[0]=20+0.001*((rand() % 10 + 1)-5); //u_{i-1}

            iderror=modelP2.UpdateSystem(cs[0],pitch);


            probe.pushBack(pitch*180/M_PI);
            probe2.pushBack(cs[0]);
            //Gz.PrintParamsVector();

            v_lengths[0]=0.001*( cs[0] / 1.5);
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

        modelP2.GetSystemBlock(sysP2);
        gainP2=sysP2.GetZTransferFunction(numP2,denP2);
        sysP2.PrintZTransferFunction(dts);
        probe.Plot();
        probe2.Plot();



    } else{

    double interval=6; //in seconds
    string sNum="";
    string sDen="";


    for (cs[0] =-ang[0] ; cs[0] <= ang[0] ; cs[0]= cs[0]+10)
    {
        for (cs[1] = -ang[1] ; cs[1] <= ang[1] ; cs[1]= cs[1]+10)
        {

            OnlineSystemIdentification modelP(numOrder, denOrder );
            OnlineSystemIdentification modelP2 (numOrder2, denOrder2 );

            OnlineSystemIdentification modelY(numOrder, denOrder );
            OnlineSystemIdentification modelY2 (numOrder2, denOrder2 );

            ofstream data("/home/humasoft/code/Soft-Arm/graphs/Identificacion/IndentificacionV3_P"+to_string(int(cs[0]))+"_Y"+to_string(int(cs[1]))+".csv",std::ofstream::out); // /home/humasoft/code/graficas

            for (double t=0;t<interval; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);

                probe.pushBack(pitch*180/M_PI);

                modelP.UpdateSystem(cs[0], pitch*180/M_PI);
                modelY.UpdateSystem(cs[1], yaw*180/M_PI);

                modelP2.UpdateSystem(cs[0], pitch*180/M_PI);
                modelY2.UpdateSystem(cs[1], yaw*180/M_PI);

                v_lengths[0]=0.001*( cs[0] / 1.5);
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
            cout <<"Done:"<<endl;
            cout<< "Alpha:"<<cs[0]<< ";   Yaw:"<<cs[1] <<endl;

            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);

            for (double t=0;t<5; t+=dts)
            {
                misensor.GetPitchRollYaw(pitch,roll,yaw);
                probe.pushBack(pitch*180/M_PI);
                Ts.WaitSamplingTime();
            }

            modelP.GetSystemBlock(sysP);
            gainP=sysP.GetZTransferFunction(numP,denP);

            modelY.GetSystemBlock(sysY);
            gainY=sysY.GetZTransferFunction(numY,denY);

            modelP2.GetSystemBlock(sysP2);
            gainP2=sysP2.GetZTransferFunction(numP2,denP2);

            modelY2.GetSystemBlock(sysY2);
            gainY2=sysY2.GetZTransferFunction(numY2,denY2);

            sNum="";
            sDen="";

            for(int i = numP.size()-1; i>=0; i--)
                {
                    sNum=sNum+ to_string(gainP*numP[i])+", ";
                }



            for(int i = denP.size()-1; i>=0; i--)
                {
                    sDen=sDen+", "+ to_string(denP[i]);
                }

            data << sNum+sDen<<endl;

            sNum="";
            sDen="";

            for(int i = numP2.size()-1; i>=0; i--)
                {
                    sNum=sNum+ to_string(gainP2*numP2[i])+", ";
                }



            for(int i = denP2.size()-1; i>=0; i--)
                {
                    sDen=sDen+", "+ to_string(denP2[i]);
                }

            data << sNum+sDen<<endl;

            sNum="";
            sDen="";

            for(int i = numY.size()-1; i>=0; i--)
                {
                    sNum=sNum+ to_string(gainY*numY[i])+", ";
                }
            for(int i = denY.size()-1; i>=0; i--)
                {
                    sDen=sDen+", "+ to_string(denY[i]);
                }

            data << sNum+sDen<<endl;

            sNum="";
            sDen="";

            for(int i = numY2.size()-1; i>=0; i--)
                {
                    sNum=sNum+ to_string(gainY2*numY2[i])+", ";
                }
            for(int i = denY2.size()-1; i>=0; i--)
                {
                    sDen=sDen+", "+ to_string(denY2[i]);
                }

            data << sNum+sDen<<endl;


        }
        probe.Plot();
    }
    }

    m1.SetPosition(0);
    m2.SetPosition(0);
    m3.SetPosition(0);
//    sleep(4);
}

