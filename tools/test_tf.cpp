#include <complex>

#include "fcontrol.h"
#include <math.h>

#include "IPlot.h"

using namespace std;

int main()
{

    vector<double>  num(5);
    vector<double>  den(5);

    double dts=0.02;
    num={-0.089862194815923,0.281419410260342,-0.293360525912833,0.101803394370491,0};
    den={0.060438759633418,-1.1333102350899,3.085117496232294,-3.012245968645809,1};

    SystemBlock fPDp(num,den);

    num={-0.07640859291221,0.238508579019849,-0.247871985677167,0.085772060066429,0};

    den={0.041725532270869,-1.078494548630914,3.031640050655836,-2.994870988390003,1};
    SystemBlock fPDy(num,den);

    fPDp.PrintZTransferFunction(dts);
    fPDy.PrintZTransferFunction(dts);


    IPlot realp(dts,"Realp","t(s)","Out");
    IPlot realy(dts,"Realy","t(s)","Out");

    IPlot id(dts,"Identication","t(s)","Out");

    double in=0,out=0;
    double tmax=500;
    double outp;    double outy;
    long iderrors=0;


    for (double t=0; t<tmax; t+=dts)

    {

        in=20; //u_{i-1}
        outp=in > fPDp ;//y_{i}
        outy=in > fPDp ;//y_{i}

        realp.pushBack(outp);
        realy.pushBack(outy);


    }
//    real.SetParameters("set lc 'blue'");
    realp.Plot();
    realy.Plot();

    return 0;
}
