#include"phd_filter.h"

Particle phd_filter::DetectedObjectUpdate(Particle target,vec z,PHDupdate detected_update){
    Particle detected_target;
    //double c=as_scalar(mean(normpdf(z,detected_update.eta,diagvec(detected_update.S))));
    detected_target.weight=p_d_*target.weight*as_scalar(pow(det(2*datum::pi*detected_update.S),-0.5)*exp(-0.5*(z-detected_update.eta).t()*detected_update.S.i()*(z-detected_update.eta)));
    detected_target.state=target.state+detected_update.K*(z-detected_update.eta);
    detected_target.P=target.P;
    //cout<<"Eta "<<detected_update.eta<<endl;
    //cout<<"z_ "<<z<<endl;
    return detected_target;
}