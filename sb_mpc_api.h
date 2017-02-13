#ifndef SB_MPC_API_H_
#define SB_MPC_API_H_

#include "obstacle.h"
#include "sb_mpc.h"
#include "ship_model.h"

#include "Eigen\Dense"
#include "iostream"

void getControlOffset(double u_d, double psi_d, double &u_os, double &psi_os);

#endif