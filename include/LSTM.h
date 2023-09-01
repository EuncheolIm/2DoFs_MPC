#pragma once
#ifndef LSTM_H
#define LSTM_H

#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h>

using namespace std;
using namespace Eigen;

class LSTM
{
    
public:
    LSTM();
    virtual ~LSTM();
    void LSTM_layer();
    VectorXd Fc_layer(int num_output);
    void state_update(VectorXd x);
    void setup_weight(const char weight_hh[], const char weigh_hi[], const char bias_hh[], const char bias_hi[], const char fcWeight1[], const char fcbias1[]);

    void buffer_update(MatrixXd x);

    void setup_weight_each_joint(const char weight1[], const char weight2[], const char bias1[], const char bias2[], const char fcWeight1[],const char fcbias1[]);
    void LSTM_layer_each_joint();
    double Fc_layer_j1();
private:
    void Initialize();
    VectorXd V_Sigmoid(VectorXd x);
    VectorXd V_Csinh(VectorXd x);
    VectorXd V_Ccosh(VectorXd x);
    VectorXd V_Ctanh(VectorXd x);
    VectorXd Sigmoid(VectorXd x);

    ifstream weight;
    ////    pred touque j1, j4
    /// LSTM weight + bias ///
    double weight0[64][6];     //W_ih
    double weight1[64][16];    //W_hh 
    double weight2[64];        //b_ih
    double weight3[64];        //b_hh
    /// LSTM weight + bias ///
    double weight4[2][16];     //W_fc
    double weight5[2];         //b_fc

    ////    pred touque j1
    double weight_j1_0[64][4];     //W_ih
    double weight_j1_1[64][16];    //W_hh 
    double weight_j1_2[64];        //b_ih
    double weight_j1_3[64];        //b_hh
    /// LSTM weight + bias ///
    double weight_j1_4[1][16];     //W_fc
    double weight_j1_5[1];         //b_fc


    int features, hidden_units, fc_node, num_classes, timewindow;
    int num_class;

    VectorXd x, A;
    MatrixXd x_buffer;
    MatrixXd Wi, Wh;
    VectorXd bias, _Fcb1;
    MatrixXd _FcW1, buffer;

    MatrixXd _FcW1_j1;
    VectorXd _Fcb1_j1;

    VectorXd _ft, _it, _ot, _gt, _V_ct, ft_ct_1, it_gt;
    VectorXd _Ct_1, _Ct, _ht_1, _ht;
    VectorXd Fc1_output, output;
    VectorXd Fc1_output_j1, output_j1;
    // double output_j1;

    MatrixXd Wii, Wif, Wig, Wio;
    MatrixXd Whi, Whf, Whg, Who;
    VectorXd bii, bif, big, bio;
    VectorXd bhi, bhf, bhg, bho;

    MatrixXd Wii_j1, Wif_j1, Wig_j1, Wio_j1;
    MatrixXd Whi_j1, Whf_j1, Whg_j1, Who_j1;
    VectorXd bii_j1, bif_j1, big_j1, bio_j1;
    VectorXd bhi_j1, bhf_j1, bhg_j1, bho_j1;

    ofstream ffout;
    
};
#endif