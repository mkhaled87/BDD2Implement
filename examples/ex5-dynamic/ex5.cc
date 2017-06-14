
#include <vector>
#include <iostream>
#include "BDD2Implement.hh"

#define ssDIM 1
#define isDIM 1
#define cs1DIM 1
#define cs2DIM 1

#define BDD_FILE "bdd/dynamic.bdd"
#define DCONTR_FILE "vehicle_contr_determinized.bdd"
#define VHDL_FILE "bdd_vehicle.vhdl"

int main() {
	Cudd mgr;

	BDDReader reader(mgr, BDD_FILE, BDD_FILE_TYPE::SCOTS_BDD);
	BDD contr = reader.ReadBdd();
    	size_t nBddvars_states  = reader.getNumBddVars(0, ssDIM);
	size_t nBddvars_contr_current_states= reader.getNumBddVars(ssDIM,cs1DIM);
	size_t nBddvars_actions = reader.getNumBddVars(cs1DIM, isDIM);
	size_t nBddvars_contr_next_states = reader.getNumBddVars(isDIM,cs2DIM);

    	size_t nBddvars_states_dynamic=nBddvars_states+nBddvars_contr_current_states;
    	size_t nBddvars_actions_dynamic=nBddvars_actions+nBddvars_contr_next_states;

	cout << "nBddvars_states : " << nBddvars_states << endl;
	cout << "nBddvars_contr_current_states : " << nBddvars_contr_current_states << endl;
	cout << "nBddvars_contr_next_states : " << nBddvars_contr_next_states << endl;
	cout << "nBddvars_actions: " << nBddvars_actions << endl;

	BddDeterminizer determinizer(mgr, contr, nBddvars_states_dynamic,nBddvars_actions_dynamic);
	contr = determinizer.determinize(DETERMINIZE_METHOD::RANDOM, true, 1);

	if(contr == mgr.bddZero()){
	    cout << "Error: empty determinized controller";
	    return 0;
	}

	cout << "Saving the determinized controller ... ";

	SCOTSBDDReader scots_reader = reader.getScotsReader();
	scots_reader.setBDD(contr);
	scots_reader.writeToFile(DCONTR_FILE);

	BddToHDLdynamic converter (mgr, contr, nBddvars_states, nBddvars_contr_current_states, nBddvars_contr_next_states,nBddvars_actions);
	converter.GenerateRawVHDL(VHDL_FILE, true, 1);
	cout << "done !";
	return 1;
}
