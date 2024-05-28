#ifndef GAMMA_PROJ_HPP_
#define GAMMA_PROJ_HPP_
#include "type_definitions.hpp"

class GammaPrj{

    public:

        GammaPrj();

        GammaPrj(mat33_t& Gamma);

        void getProjGamma(mat31_t vec, mat31_t y, 
        double f, mat31_t grad_f ,mat31_t& vec_proj);

    private:

        mat33_t Gamma_;

};


#endif