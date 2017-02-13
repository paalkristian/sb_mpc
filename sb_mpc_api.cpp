#include "sb_mpc_api.h"
/*
	In the final implementation I need to take in ASV state and obstacle states as well,
	and generate the appropriate matrices from the parameters.
*/
void getControlOffset(double u_d, double psi_d, double &u_os, double &psi_os) {
	SimulationBasedMpc *sb_mpc = new SimulationBasedMpc();

	Eigen::Matrix<double, 6, 1> asv_state;
	asv_state << 0, 0, -0.0959975, 6.88277, 0, 0;

	Eigen::Matrix<double, 5, 9> obst_states;
	obst_states << 110.359, 146.154, 4.71239, 3, 0, 10, 10, 10, 10,
		-826.865, 714.124, 0, 0, 0, 10, 10, 10, 10,
		3444.27, -1950.35, 5.72293, 13.2727, 0, 10, 10, 10, 10,
		-614.694, 735.825, 1.24093, 3.29244, 0, 10, 10, 10, 10,
		-522.324, 6325.05, 2.27242, 0, 0, 10, 10, 10, 10;

	sb_mpc->getBestControlOffset(u_os, psi_os, u_d, psi_d, asv_state, obst_states);
}