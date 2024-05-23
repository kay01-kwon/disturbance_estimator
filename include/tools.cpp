#include "tools.hpp"

void get_dqdt(quat_t q, mat31_t w, quat_t& dqdt)
{
    quat_t w_quat;
    quat_t q_otimes_w;

    w_quat.w() = 0;
    w_quat.x() = w(0);
    w_quat.y() = w(1);
    w_quat.z() = w(2);

    otimes(q, w_quat, q_otimes_w);

    dqdt.w() = 0.5*q_otimes_w.w();
    dqdt.x() = 0.5*q_otimes_w.x();
    dqdt.y() = 0.5*q_otimes_w.y();
    dqdt.z() = 0.5*q_otimes_w.z();
}


/**
 * Conjugate of the quaternion
 * param: quat_t q, quat_t& q_res
 * q_res is the conjugate of the quaternion q.
*/
void conjugate(quat_t q, quat_t& q_res)
{
    q_res.w() = q.w();
    q_res.x() = -q.x();
    q_res.y() = -q.y();
    q_res.z() = -q.z();
}

/**
 * The multiplication of two quaternion q1 and q2
 * param: quat_t q1, quat_t q2, quat_t& q_res
 * q_res is the multiplication of q1 and q2.
*/
void otimes(quat_t q1, quat_t q2, quat_t& q_res)
{
    double real;
    mat31_t q1_vec, q2_vec;
    mat33_t q1_skiew_sym_mat;
    mat31_t q_res_vec;

    quat2quat_vec(q1, q1_vec);
    quat2quat_vec(q2, q2_vec);

    vec2skiew(q1_vec, q1_skiew_sym_mat);
    
    real = q1.w() * q2.w() 
    - q1_vec.transpose()*q2_vec;

    q_res_vec = q1.w()*q2_vec 
    + q2.w()*q1_vec
    + q1_skiew_sym_mat*q2_vec;

    q_res.w() = real;
    q_res.x() = q_res_vec(0);
    q_res.y() = q_res_vec(1);
    q_res.z() = q_res_vec(2);
}

/**
 * get rotation matrix from the quaternion
 * param: quat_t q, mat33_t& rotm_res
 * rotm_res is 3 by 3 rotation matrix.
*/

void get_Rotm_from_quat(quat_t q, mat33_t& rotm_res)
{
    mat33_t eye_m;
    mat31_t q_vec;
    mat33_t skiew_sym;

    eye_m.setIdentity();

    quat2quat_vec(q, q_vec);
    vec2skiew(q_vec,skiew_sym);

    rotm_res = (q.w()*q.w() - q_vec.transpose()*q_vec)*eye_m
    + 2 * q_vec * q_vec.transpose()
    + 2 * q.w() * skiew_sym;
}

/**
 * Convert 3 by 1 vector to 3 by 3 skiew symmetric matrix
 * param: mat31_t vec, mat33_t& skiew_sym_mat
*/
void vec2skiew(mat31_t vec, mat33_t& skiew_sym_mat)
{
    skiew_sym_mat << 0, -vec(2), vec(1),
                    vec(2), 0, -vec(0),
                    -vec(1), vec(0), 0;
}

/**
 * Convert 4 by 1 quaternion to 3 by 1 
 * vector of the quaternion.
*/
void quat2quat_vec(quat_t q, mat31_t& q_vec)
{
    q_vec << q.x(), q.y(), q.z();
}

/**
 * Normalizing quaternion
*/
void quat2unit_quat(quat_t q, quat_t &unit_q)
{
    double den;

    den = sqrt(q.x() * q.x()
    + q.y() * q.y()
    + q.z() * q.z()
    + q.w() * q.w()
    );

    unit_q.x() = q.x()/den;
    unit_q.y() = q.y()/den;
    unit_q.z() = q.z()/den;
    unit_q.w() = q.w()/den;
}

/**
 * Signum function
*/
double signum(double num)
{
    return num > 0 ? 1.0:-1.0;
}

void print_params(Inertial_param& param1, Aero_coeff& param2, double& param3)
{
    cout<<"Inertial Parameter setup"<<endl;

    cout<<"Mass: ";
    cout<<param1.m<<endl;

    cout<<"MOI: ";
    cout<<param1.J<<endl;

    cout<<"Aerodynamics coefficients setup "<<endl;
    cout<<"Lift coefficient: ";
    cout<<param2.lift_coeff<<endl;

    cout<<"Moment coefficient:";
    cout<<param2.moment_coeff<<endl;

    cout<<"Arm length: "<<param3<<endl;
}

double noise(double stddev, long long seedNum)
{
    mt19937::result_type seed = seedNum;
    auto dist = std::bind(
        std::normal_distribution<double>{0, stddev},
        mt19937(seed));

    return dist();
}

long long get_seedNum()
{
    auto time_stamp = system_clock::now();
    auto duration = time_stamp.time_since_epoch();
    auto milisec = 
    std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return static_cast<long long>(milisec);
}

void demux_vec3(mat31_t v, 
vector<double>&x, 
vector<double>&y, 
vector<double>&z)
{
    x.push_back(v(0));
    y.push_back(v(1));
    z.push_back(v(2));
}