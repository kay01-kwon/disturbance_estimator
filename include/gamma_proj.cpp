#include "gamma_proj.hpp"

GammaPrj::GammaPrj()
{
    Gamma_.setIdentity();
    Gamma_ *= 1000.0;
    cout << "Value for Gamma:" << endl;
    cout << Gamma_ <<endl;
}

GammaPrj::GammaPrj(mat33_t &Gamma): Gamma_(Gamma)
{
    cout << "Value for Gamma:" << endl;
    cout << Gamma_ <<endl;
}

void GammaPrj::getProjGamma(mat31_t vec, mat31_t y, double f, mat31_t grad_f, mat31_t& vec_proj)
{
    if( (f > 0) && ( y.transpose()*Gamma_*grad_f > 0 ))
    {
        vec_proj = Gamma_*(
            y - 
            grad_f*grad_f.transpose()/(grad_f.transpose()*grad_f)
            *Gamma_*y*f
        );
    }
    else
    {
        vec_proj = Gamma_*y;
    }
}