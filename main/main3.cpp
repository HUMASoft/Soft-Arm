#include "Cia402device.h"
#include "CiA301CommPort.h"
#include "SocketCanPort.h"
#include <iostream>
#include <stdio.h>
#include <Kinematics.h>
#include "fcontrol.h"
#include "IPlot.h"
#include "imu3dmgx510.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <vector>

int connectClient();
int startRecording();
int stopRecording();

NatNetClient* g_pClient = NULL;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;
int g_analogSamplesPerMocapFrame = 0;
static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;


int main ()
{

// print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "Trigger OptiTrack Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );
    g_pClient = new NatNetClient();
    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverAddress = "2.2.2.107";

    int iResult;
    iResult = connectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client. Exiting\n");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }

    vector<double> ang(2);
    ang[0] = 20; //ALPHA
    ang[1] = 0; //BETA

    cout<< "456"<<endl;

    ofstream data("/home/humasoft/code/Soft-Arm/graphs/MocapTest20min2.csv",std::ofstream::out); // /home/humasoft/code/graficas
    //--Can port communications--

    string can = "can0";
    SocketCanPort pm1(can);
    CiA402SetupData sd1(2048,157,0.001, 1.25, 20 );
    CiA402Device m1 (31, &pm1, &sd1);
    m1.SetupPositionMode(3,3);

    SocketCanPort pm2(can);
    CiA402SetupData sd2(2048,157,0.001, 1.25, 20 );
    CiA402Device m2 (32, &pm2, &sd2);    //--Can port communications--
    m2.SetupPositionMode(3,3);

    SocketCanPort pm3(can);
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

    //plot
    IPlot probe(dts,"Plot Pitch");
    IPlot probe1(dts,"Plot Yaw");
    IPlot probe2(dts,"Plot CPitch");
    IPlot probe3(dts,"Plot m2");
    IPlot probe4(dts,"Plot m3");

    vector<double> ierror(2); // ERROR
    vector<double> cs(2); //CONTROL SIGNAL
    vector<double> valores(8);
    valores[0]=40;
    valores[1]=0;
    valores[2]=0;
    valores[3]=0;
    valores[4]=0;
    valores[5]=20;
    valores[6]=40;
    valores[7]=-40;

    //TEST
    valores[0]=ang[0];
    valores[4]=ang[1];


    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_streamon();

    for (double t=0;t<10;t+=dts)
    {
        misensor.GetPitchRollYaw(pitch,roll,yaw);
        //cout<<"Calibrando"<<endl;
        //cout << "Roll: " << roll*180/M_PI << " Pitch: " <<pitch*180/M_PI  << " Yaw: " << yaw*180/M_PI << endl;
    }
    cout<<"Calibrado"<<endl;


    iResult = startRecording();

    if (iResult != ErrorCode_OK)
        {
            printf("Error sending 'rec' instruction. Exiting");
            return 1;
        }
        else
        {
            printf("Client send rec instruction successfully.\n");
        }

    double interval=600; //in seconds
    for (double t=0;t<interval; t+=dts)
    {
        cs[0]=20*(sin(t)+sin(t/4))+3*double(rand())/(RAND_MAX);//1*(sin(t)+sin(t/4))+ rand()*3;
        cs[1]=20*(sin(t/3)+sin(t/7))+3*double(rand())/(RAND_MAX);



            misensor.GetPitchRollYaw(pitch,roll,yaw);

            probe.pushBack(pitch*180/M_PI);
            probe1.pushBack(yaw*180/M_PI);
            probe2.pushBack(cs[0]);
            probe3.pushBack(cs[1]);

            if (!isnormal(cs[0])) cs[0] = 0;

            if (!isnormal(cs[1])) cs[1] = 0;


            v_lengths[0]=0.001*( cs[0] / 1.5);
            v_lengths[1]=0.001*( - (cs[0] / 3) - (cs[1] / 1.732) );
            v_lengths[2]=0.001*( (cs[1] / 1.732) - (cs[0] / 3) );

            posan1=(v_lengths[0])/radio;
            posan2=(v_lengths[1])/radio;
            posan3=(v_lengths[2])/radio;


//            probe4.pushBack(posan2);

            m1.SetPosition(posan1);
            m2.SetPosition(posan2);
            m3.SetPosition(posan3);

            data << roll << " , " << pitch << " , " << yaw<<" , " <<  m1.GetPosition() <<" , " <<m2.GetPosition() <<" , " <<m3.GetPosition() <<" , " <<  m1.GetVelocity() <<" , " <<m2.GetVelocity() <<" , " <<m3.GetVelocity() <<" , " <<  m1.GetAmps() <<" , " <<m2.GetAmps() <<" , " <<m3.GetAmps() << " , " << cs[0] << " , " <<cs[1] << endl; //CR
            //cout << endl;
            Ts.WaitSamplingTime();
        }
        interval=3;

        iResult = stopRecording();

        if (iResult != ErrorCode_OK)
        {
            printf("Error sending 'stop' instruction. Exiting");
            return 1;
        }
        else
        {
            printf("Client send stop instruction successfully.\n");
        }

        for (double t=0;t<interval; t+=dts)
        {
            misensor.GetPitchRollYaw(pitch,roll,yaw);

            probe.pushBack(pitch*180/M_PI);
            probe1.pushBack(yaw*180/M_PI);
            probe2.pushBack(0);
            probe3.pushBack(0);

            m1.SetPosition(0);
            m2.SetPosition(0);
            m3.SetPosition(0);
            Ts.WaitSamplingTime();
        }

    cout <<"Done" << endl;
    probe.Plot();
    probe1.Plot();
    probe2.Plot();
    probe3.Plot();
//    probe4.Plot();


}

// Establish a NatNet Client connection
int connectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    return ErrorCode_OK;
}

int startRecording()
{
    void* pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;
    std::string command = "StartRecording";

    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded
        ret = g_pClient->SendMessageAndWait("StartRecording", 3, 100, &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            int opResult = *((int*)pResult);
            if (opResult == 0)
                printf("%s handled and succeeded.\n", command.c_str());
            else
                printf("&%s handled but failed.\n", command.c_str());
        }
        else
            printf("Error sending command: %s.\n", command.c_str());
    }
    return ErrorCode_OK;
}

int stopRecording()
{
    void* pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;
    std::string command = "StopRecording";

    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting\n", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded
        ret = g_pClient->SendMessageAndWait(command.c_str(), 3, 100, &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            int opResult = *((int*)pResult);
            if (opResult == 0)
                printf("%s handled and succeeded.\n", command.c_str());
            else
                printf("&%s handled but failed.\n", command.c_str());
        }
        else
            printf("Error sending command: %s.\n", command.c_str());
    }
    return ErrorCode_OK;
}
