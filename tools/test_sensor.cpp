//Used libs in the project

#include <iostream>
#include <ios>

#include <boost/asio.hpp> // include boost
#include <boost/asio/serial_port.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <string.h>
#include <math.h>
#include <sstream>
#include <boost/algorithm/hex.hpp>
#include "imu3dmgx510.h"


#include <tuple>
//#include <yarp/os/Bottle.h>


//using namespace std;
using namespace boost::asio;
using namespace boost::algorithm;
using namespace std::string_literals;
using namespace stateestimation;
using std::cin;
using std::cout;

// Port is defined depending on the OS used by the user
// These are the values our port needs to connect
#ifdef _WIN32
// windows uses com ports, this depends on what com port your cable is plugged in to.
const char *PORT = "COM7";
#else
//default port usb
const char *PORT = "/dev/ttyUSB0";
#endif

//Plotting functions are only used to copy-paste vector in Matlab

void PlotEulerAngles(double* rollangle,double* pitchangle, double* yawangle, double rollaverage, double pitchaverage,double yawaverage , int numero){ //CR

    cout << "Initial offset in pitch is: "<< pitchaverage << '\n';
    cout << "Initial offset in roll is: "<< rollaverage << '\n';
    cout << "Initial offset in yaw is: "<< yawaverage << '\n'; //CR

    for (int c=0;c<=numero-100;c++){

        //Jump corrections in case device is placed face up. Uncomment it if device is placed face up.
        //                        if (c>200 && abs(*(roll+c)-*(roll+c-1))>5){
        //                            *(roll+c)=*(roll+c)+6;
        //                        }

        //                        if (*(roll+c)<=-3.2){
        //                            *(roll+c)=*(roll+c)+6;
        //                            }


        if (c==0){
            cout << "roll = [" << *(rollangle+c) << " ";
        }
        if (c==(numero-100)){
            cout << *(rollangle+c) << ']'<< '\n';
        }else{
            cout << *(rollangle+c) << " ";
        }

    }
    for (int c=0;c<=numero-100;c++){
        if (c==0){
            cout << "pitch = [" << *(pitchangle+c) << " ";
        }
        if (c==(numero-100)){
            cout << *(pitchangle+c) << ']'<< '\n';
        }else{
            cout << *(pitchangle+c) << " ";
        }
    }
    //CR  {
    for (int c=0;c<=numero-100;c++){
        if (c==0){
            cout << "yaw = [" << *(yawangle+c) << " ";
        }
        if (c==(numero-100)){
            cout << *(yawangle+c) << ']'<< '\n';
        }else{
            cout << *(yawangle+c) << " ";
        }
    }
    //CR   }

}
void PlotGyro(double* gyrosx,double* gyrosy, double* gyrosz, int numero){

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyrox = [" << *(gyrosx+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosx+c) << ']'<< '\n';
        }else{
            cout << *(gyrosx+c) << " ";
        }

    }

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyroy = [" << *(gyrosy+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosy+c) << ']'<< '\n';
        }else{
            cout << *(gyrosy+c) << " ";
        }

    }

    for (int c=0;c<=numero-100;c++){

        if (c==0){
            cout << "gyroz = [" << *(gyrosz+c) << " ";
        }
        if (c==(1000-100)){
            cout << *(gyrosz+c) << ']'<< '\n';
        }else{
            cout << *(gyrosz+c) << " ";
        }

    }


}


int main()
{

//    IMU3DMGX510 misensor ("COM7");
    IMU3DMGX510 misensor("/dev/ttyUSB0");

    int end=0;
    double *roll;
    double *pitch;
    double *yaw; //CR
    double absrollaverage=0.0;
    double abspitchaverage=0.0;

    double absyawaverage=0.0; //CR

    double *gyrox;
    double *gyroy;
    double *gyroz;
    float gyroxvalue;
    float gyroyvalue;
    float gyrozvalue;
    double *estimator;
    double *EulerAngles;
    cout <<"10"<< endl;


    //Calibration
    misensor.set_IDLEmode();
    cout <<"20"<< endl;
    misensor.set_freq(100);
    cout <<"30"<< endl;
    misensor.set_devicetogetgyroacc();
    cout <<"40"<< endl;
    misensor.set_streamon();
    cout << "Calibrating IMU..." << endl;
    misensor.calibrate();
    cout << "Calibration done" << endl;

    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    misensor.set_IDLEmode();

    //After it, user could be able to select the functionality of the sensor


    misensor.set_freq(100);
    misensor.set_IDLEmode();
    misensor.set_devicetogetgyroacc();
    misensor.set_streamon();

    do{
        EulerAngles = misensor.EulerAngles();
        cout << "Roll: " << EulerAngles[0] << " Pitch: " << EulerAngles[1] << " Yaw: " << EulerAngles[2]<< endl; //CR
    }while(true);

    return 0;
}



