#include <complex>

#include "fcontrol.h"
#include <math.h>

#include "IPlot.h"

using namespace std;

int main()
{



//    SystemBlock sys(vector<double>{1,1,1},vector<double>{0.5,-0.9,1});
//    SystemBlock sys(vector<double>{10},vector<double>{-199,201});

    double dts=0.01;

//    (z^2-2*z+1)/((dts^2+2*dts+2)*z^2+(2*dts^2-4)*z+dts^2-2*dts+2)
//    SystemBlock sys(vector<double>{1, -2, 1},vector<double>{dts*dts-2*dts+2, 2*dts*dts-4, dts*dts+2*dts+2});
    double p1r=0.8,p1i=0.4, p2r=0.85;

//    //complex pole pair (z-p1)(z-cp1)=z^2-2Re(p1)z+|p1|^2
//    double a0=p1r*p1r+p1i*p1i;
//    double a1=-2*p1r;
//    double a2=1;
//    SystemBlock sys(vector<double>{1},vector<double>{p1r*p1r+p1i*p1i,-2*p1r,1});

    //complex pole pair (z-p1)(z-cp1)=z^2-2Re(p1)z+|p1|^2
    //composition with real pole (z-p2r)
    // z^3  -2Re(p1)z^2 -p2r*z^2  +|p1|^2z +p2r*2Re(p1)z  -p2r*|p1|^2
    double a0=-(p1r*p1r+p1i*p1i)*p2r;
    double a1=p1r*p1r+p1i*p1i+p2r*2*p1r;
    double a2=-2*p1r-p2r;
    double a3=1;


    SystemBlock sys(vector<double>{1},vector<double>{a0,a1,a2,a3});
//    SystemBlock sys2(1,0,-p2r,1);
//    SystemBlock sys(vector<double>{1},vector<double>{-p2r*(p1r*p1r+p1i*p1i),-2*p1r,1});


    //Low system gains may result in identification errors!!!

    int numOrder=sys.GetNumOrder(),denOrder=sys.GetDenOrder();
    OnlineSystemIdentification Gz(numOrder,denOrder,0.95);

    IPlot real(dts,"t(s)","Out","real");
    IPlot id(dts,"t(s)","Out","id");

    double in=0,out=0;
    double tmax=5;
    double iderror=0;
    long iderrors=0;

    for (double t=0; t<tmax; t+=dts)

    {

        in=1+0.0001*((rand() % 10 + 1)-5); //u_{i-1}
        out=in > sys ;//y_{i}

        iderror=Gz.UpdateSystem(in,out);
        cout << "id error at "  << t << " = "<< iderror << endl;
//        if (abs() > 2)
//        {
//            cout << "id error at "  << t << endl;
//            break;
//        }

        real.pushBack(out);
        //Gz.PrintParamsVector();


    }
//    real.SetParameters("set lc 'blue'");
    real.Plot();

    Gz.PrintZTransferFunction(dts);
    sys.PrintZTransferFunction(dts);


    vector<double> num(numOrder+1),den(denOrder+1);
    Gz.GetZTransferFunction(num,den);
    SystemBlock idsys(num,den);

    for (double t=0; t<tmax; t+=dts)

    {
        in=1;//*(rand() % 10 + 1)-5;
        out=in > idsys;
        id.pushBack(out);
        //Gz.PrintZTransferFunction(dts);
        //Gz.PrintParamsVector();

    }

    id.Plot();

//    vector<double> params = Gz.GetParamsVector();
//    for (int i=0; i<params.size(); i++) cout << params[i] << endl;

//    p.Plot();

        cout << a0 << endl << a1 << endl << a2<< endl  << a3<< endl ;
    return 0;
}
