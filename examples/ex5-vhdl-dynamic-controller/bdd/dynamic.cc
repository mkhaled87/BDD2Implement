
/*generate BDD file*/
#include <iostream>
#include <array>

#include "cuddObj.hh"
#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"


#define sDIM 1
#define iDIM 1

using namespace std;

int main(){

  Cudd mgr;


  double lb_ss[sDIM]={1.0};
  double ub_ss[sDIM]={4.0};
  double eta_ss[sDIM]={1.0};
  scots::SymbolicSet ss (mgr,sDIM,lb_ss,ub_ss,eta_ss);

  double lb_cs1[sDIM]={1.0};
  double ub_cs1[sDIM]={2.0};
  double eta_cs1[sDIM]={1.0};
  scots::SymbolicSet cs1(mgr,sDIM,lb_cs1,ub_cs1,eta_cs1);

  double lb_is[iDIM]={1.0};
  double ub_is[iDIM]={2.0};
  double eta_is[iDIM]={1.0};
  scots::SymbolicSet is (mgr,iDIM,lb_is,ub_is,eta_is);

  double lb_cs2[sDIM]={1.0};
  double ub_cs2[sDIM]={2.0};
  double eta_cs2[sDIM]={1.0};
  scots::SymbolicSet cs2(mgr,sDIM,lb_cs2,ub_cs2,eta_cs2);

  scots::SymbolicSet ss_cs1(ss,cs1);
  scots::SymbolicSet ss_cs1_is(ss_cs1,is);
  scots::SymbolicSet controller(ss_cs1_is,cs2);

  size_t** p;
  p=controller.getIndBddVars();

  BDD x1 =mgr.bddVar(p[0][0]);
  BDD x2 =mgr.bddVar(p[0][1]);
  BDD x3 =mgr.bddVar(p[1][0]);
  BDD y1 =mgr.bddVar(p[2][0]);
  BDD y2 =mgr.bddVar(p[3][0]);

  BDD f=(!x1)*(!x2)*(!x3)*(!y1)*(!y2)+(!x1)*x2*(!x3)*y1*(!y2)+x1*(!x2)*(!x3)*(!y1)*(!y2)+x1*x2*(!x3)*(!y1)*y2+x1*x2*x3*(!y1)*y2+x1*(!x2)*x3*y1*y2+(!x1)*x2*x3*(!y1)*y2+(!x1)*(!x2)*x3*(!y1)*(!y2);
  /*cout<<  << endl;*/
  controller.setSymbolicSet(f);
  controller.writeToFile("dynamic.bdd");

  return 1;


}
