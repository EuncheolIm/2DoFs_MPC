#pragma once
#ifndef GRU_H
#define GRU_H

#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h>

using namespace std;
using namespace Eigen;

class GRU
{
public:
    GRU();
    virtual ~GRU();
    void GRU_layer();

    VectorXd Fc_layer(int num_output);
    void state_update(VectorXd x);
    void setup_weight(const char weight_hh[], const char weigh_hi[], const char bias_hh[], const char bias_hi[], const char fcWeight1[], const char fcbias1[]);
    void buffer_update(MatrixXd x);

private:
    void Initialize();
    VectorXd V_Sigmoid(VectorXd x);
    VectorXd V_Csinh(VectorXd x);
    VectorXd V_Ccosh(VectorXd x);
    VectorXd V_Ctanh(VectorXd x);
    VectorXd Sigmoid(VectorXd x);

    ifstream weight;

    /// GRU weight + bias ///
    /// LSTM weight + bias ///
    double weight0[96][21];     //W_ih
    double weight1[96][32];    //W_hh 
    double weight2[96];        //b_ih
    double weight3[96];        //b_hh
    /// LSTM weight + bias ///
    double weight4[7][32];     //W_fc
    double weight5[7];         //b_fc


    int features, hidden_units, fc_node, num_classes, timewindow;

    VectorXd x;
    MatrixXd x_buffer;
    VectorXd _Fcb1;
    MatrixXd _FcW1, buffer;

    VectorXd _rt, _zt, _nt, _nt_fw, _nt_pw;
    VectorXd _ht_1, _ht;
    VectorXd Fc1_output, output;

    MatrixXd Wir, Wiz, Win;
    MatrixXd Whr, Whz, Whn;
    VectorXd bir, biz, bin;
    VectorXd bhr, bhz, bhn;

    ofstream ffout;
};
#endif