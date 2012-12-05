#pragma once
#include <vector>
#include <string>
#include <gurobi_c++.h>
#include "sqp_fwd.h"
using std::vector;
using std::string;

const char* getGRBStatusString(int status);
void initializeGRB();
boost::shared_ptr<GRBEnv> getGRBEnv();

class ConvexPart {
	// constraint or cost that can be added to problem
public:
	void addToModel(GRBModel* model);
	void removeFromModel();

	ConvexPart();
	virtual ~ConvexPart();
  bool m_inModel;
  GRBModel* m_model;
	
	vector<GRBVar> m_vars;

  vector<string> m_eqcntNames; // future names for constraints
  vector<GRBLinExpr> m_eqexprs; // expressions that are == 0
  vector<GRBConstr> m_eqcnts;

	vector<string> m_cntNames; // future names for constraints
	vector<GRBLinExpr> m_exprs; // expressions that are <= 0
	vector<GRBConstr> m_cnts; // expressions get turned into constraints

  vector<string> m_qcntNames; // future names for constraints
	vector<GRBQuadExpr> m_qexprs; // expressions that are <= 0
	vector<GRBQConstr> m_qcnts; // expressions get turned into constraints

};

class ConvexObjective : public ConvexPart {
public:
	GRBQuadExpr m_objective;
};

class ConvexConstraint : public ConvexPart {
public:
};


class Cost {
public:
	virtual ConvexObjectivePtr convexify(GRBModel* model)=0;
	virtual double evaluate() = 0;
	virtual string getName() {return "Unnamed cost";}
	virtual ~Cost() {}

};

class Constraint {
public:
	virtual ConvexConstraintPtr convexify(GRBModel* model)=0;
	virtual ~Constraint() {}
};

class NonlinearConstraint : public Constraint {
public:
	virtual double evaluate() {return 0;}
	virtual string getName() {return "Unnamed constraint";}
};

class TrustRegion {
public:
	double m_shrinkage;
	TrustRegion();
	virtual void adjustTrustRegion(double ratio) = 0;
	void resetTrustRegion();
  virtual ConvexConstraintPtr convexConstraint(GRBModel*) = 0;
  virtual ConvexObjectivePtr convexObjective(GRBModel*) = 0;
  virtual ~TrustRegion() {}
};

class Optimizer {
public:

	enum OptStatus {
		CONVERGED,
  	SHRINKAGE_LIMIT, // trust region shrunk too much, but convergence might not have happened
		ITERATION_LIMIT, // hit iteration limit before convergence
		GRB_FAIL, // gurobi failed
    PROGRAMMER_ERROR // your gradients/convexification are wrong
	};
	
	vector<CostPtr> m_costs;
	vector<ConstraintPtr> m_cnts;	
	TrustRegionPtr m_tra;
	GRBModel* m_model;
	
	Optimizer();
	virtual ~Optimizer();
	
	virtual void updateValues() = 0;
	virtual void storeValues() = 0;
	virtual void rollbackValues() = 0;

	virtual void preOptimize() {} // do plots and stuff here
	virtual void postOptimize() {} // ditto
	virtual void fixVariables() {assert(0);}
	OptStatus optimize();
  void addCost(CostPtr cost);
  void addConstraint(ConstraintPtr cnt);
  void setTrustRegion(TrustRegionPtr tra);

protected:

  int convexOptimize();
	double getApproxObjective();
	vector<ConvexObjectivePtr> convexifyObjectives();
	vector<ConvexConstraintPtr> convexifyConstraints();
	void printObjectiveInfo(const vector<double>& oldExact, const vector<double>& newApprox, const vector<double>& newExact);
	void printConstraintInfo(const vector<double>& oldExact, const vector<double>& newExact);
	void setupConvexProblem(const vector<ConvexObjectivePtr>&, const vector<ConvexConstraintPtr>&);
	void clearConvexProblem(const vector<ConvexObjectivePtr>&, const vector<ConvexConstraintPtr>&);

};
typedef vector<GRBLinExpr> ExprVector;
void addNormCost(ConvexObjectivePtr& cost, double coeff, const ExprVector& err, GRBModel* model, const string& desc);
void addHingeCost(ConvexObjectivePtr& cost, double coeff, const GRBLinExpr& err, GRBModel* model, const string& desc);
void addAbsCost(ConvexObjectivePtr& cost, double coef, const GRBLinExpr& err, GRBModel* model, const string& desc);
