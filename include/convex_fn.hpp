#ifndef CONVEX_FN_HPP_
#define CONVEX_FN_HPP_
#include "type_definitions.hpp"

class ConvFn{

    public:

        ConvFn();

        ConvFn(const mat31_t& bound,
        double epsilon_);

        void get_fn_value(mat31_t vec,
        double& f, mat31_t& Df);

        ~ConvFn();

    private:

        double bound_scalar_;
        double epsilon_;
};


#endif