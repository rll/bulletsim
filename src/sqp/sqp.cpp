#include "sqp.h"
#include "config_sqp.h"
#include "utils/logging.h"
#include <boost/format.hpp>
#include <cmath>

static const char* grb_statuses[] = { "XXX", //0
    "LOADED",//1
    "OPTIMAL",//2
    "INFEASIBLE",//3
    "INF_OR_UNBD",//4
    "UNBOUDNED",//5
    "CUTOFF",//6
    "ITERATION LIMIT",//7
    "NODE_LIMIT",//8
    "TIME_LIMIT",//9
    "SOLUTION_LIMIT",//10
    "INTERRUPTED",//11
    "NUMERIC",//12
    "SUBOPTIMAL" //13
    };

static boost::shared_ptr<GRBEnv> grbEnv;
void initializeGRB() {
 grbEnv.reset(new GRBEnv());
 if (GeneralConfig::verbose > 0) grbEnv->set(GRB_IntParam_OutputFlag, 0);
}
boost::shared_ptr<GRBEnv> getGRBEnv() {
  return grbEnv;
}

const char* getGRBStatusString(int status) {
  assert(status >= 1 && status <= 13);
  printf("Grb status; %i\n",status);
  return grb_statuses[status];
}

inline double sum(const vector<double>& x) {
  double out=0;
  for (int i=0; i < x.size(); ++i) out += x[i];
  return out;
}

vector<double> evalApproxObjectives(const vector<ConvexObjectivePtr>& in) {
  vector<double> out(in.size());
  for (int i=0; i < in.size(); ++i) out[i] = in[i]->m_objective.getValue();
  return out;
}

void Optimizer::printObjectiveInfo(const vector<double>& oldExact,
    const vector<double>& newApprox, const vector<double>& newExact) {
//  LOG_INFO("cost | exact | approx-improve | exact-improve | ratio");
  LOG_INFO_FMT("%15s | %10s | %10s | %10s | %10s", "cost", "oldexact", "dapprox", "dexact", "ratio");
  for (int i=0; i < m_costs.size(); ++i) {
    double approxImprove = oldExact[i] - newApprox[i];
    double exactImprove = oldExact[i] - newExact[i];
    string fmtstr = fabs(approxImprove) < 1e-8 ? "%15s | %10.3e | %10.3e | %10.3e | (%8.3e)" : "%15s | %10.3e | %10.3e | %10.3e | %10.3e";
    LOG_INFO_FMT(fmtstr.c_str(), m_costs[i]->getName().c_str(),
                 oldExact[i], approxImprove, exactImprove, exactImprove/approxImprove);
  }
}

void Optimizer::printConstraintInfo(const vector<double>& oldExact, const vector<double>& newExact) {
  LOG_INFO_FMT("%15s | %10s | %10s", "constraint", "oldexact", "newexact");
  for (int i=0; i < m_cnts.size(); ++i) {
  	NonlinearConstraint* maybeNLC = dynamic_cast<NonlinearConstraint*>(m_cnts[i].get());
    if (maybeNLC) {
      LOG_INFO_FMT("%15s | %10.3e | %10.3e", maybeNLC->getName().c_str(), oldExact[i], newExact[i]);
    }
  }
}

ConvexPart::ConvexPart() : m_inModel(false), m_model(NULL) {}

void ConvexPart::addToModel(GRBModel* model) {
  m_model = model;
  m_cnts.reserve(m_exprs.size());
  for (int i = 0; i < m_exprs.size(); ++i) {
    m_cnts.push_back(m_model->addConstr(m_exprs[i] <= 0, m_cntNames[i].c_str()));
  }
  for (int i = 0; i < m_eqexprs.size(); ++i) {
    m_eqcnts.push_back(m_model->addConstr(m_eqexprs[i] == 0, m_eqcntNames[i].c_str()));
  }
  for (int i = 0; i < m_qexprs.size(); ++i) {
    m_qcnts.push_back(m_model->addQConstr(m_qexprs[i] <= 0, m_qcntNames[i].c_str()));
  }
  m_inModel = true;
}

void ConvexPart::removeFromModel() {
  BOOST_FOREACH(GRBConstr& cnt, m_eqcnts) {
    m_model->remove(cnt);
  }
  BOOST_FOREACH(GRBConstr& cnt, m_cnts) {
    m_model->remove(cnt);
  }
  BOOST_FOREACH(GRBQConstr& qcnt, m_qcnts) {
    m_model->remove(qcnt);
  }
  BOOST_FOREACH(GRBVar& var, m_vars) {
    m_model->remove(var);
  }
  m_inModel = false;
}

ConvexPart::~ConvexPart() {
  if (m_inModel) LOG_WARN("ConvexPart deleted but not removed from model");
}

TrustRegion::TrustRegion() : m_shrinkage(1) {}

void TrustRegion::resetTrustRegion() {
  adjustTrustRegion(1./m_shrinkage);
}

Optimizer::Optimizer() {
  m_model = new GRBModel(*getGRBEnv());
}

Optimizer::~Optimizer() {
  delete m_model;
}

double Optimizer::getApproxObjective() {
  return m_model->get(GRB_DoubleAttr_ObjVal);
}

/*
 void Optimizer::updateValues() {
 BOOST_FOREACH(ValVarMap::value_type& valvar, m_val2var) {
 *m_val2var[valvar.first] = m_val2var.second.get(GRB_DoubleAttr_X);
 }
 }
 */
vector<ConvexObjectivePtr> Optimizer::convexifyObjectives() {
  vector<ConvexObjectivePtr> out;
  BOOST_FOREACH(CostPtr& cost, m_costs) {
    out.push_back(cost->convexify(m_model));
  }
  return out;
}

vector<ConvexConstraintPtr> Optimizer::convexifyConstraints() {
  vector<ConvexConstraintPtr> out;
  BOOST_FOREACH(ConstraintPtr& cnt, m_cnts) {
    out.push_back(cnt->convexify(m_model));
  }
  return out;
}

void Optimizer::addCost(CostPtr cost) {
  m_costs.push_back(cost);
}
void Optimizer::addConstraint(ConstraintPtr cnt) {
  m_cnts.push_back(cnt);
}
void Optimizer::setTrustRegion(TrustRegionPtr tra) {
  m_tra = tra;
}

int Optimizer::convexOptimize() {
  m_model->optimize();
  return m_model->get(GRB_IntAttr_Status);
}

vector<double> evaluateObjectives(vector<CostPtr>& costs) {
  vector<double> out(costs.size());
  for (int i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->evaluate();
  }
  return out;
}

vector<double> evaluateConstraints(vector<ConstraintPtr>& cnts) {
  vector<double> out(cnts.size());
  for (int i=0; i < cnts.size(); ++i) {
  	NonlinearConstraint* maybeNLC = dynamic_cast<NonlinearConstraint*>(cnts[i].get());
    out[i] = maybeNLC ? maybeNLC->evaluate() : 0;
  }
  return out;
}

void Optimizer::setupConvexProblem(const vector<ConvexObjectivePtr>& objectives, const vector<
    ConvexConstraintPtr>& constraints) {
  m_model->update();
  BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
    part->addToModel(m_model);
  GRBQuadExpr objective(0);
  BOOST_FOREACH(const ConvexObjectivePtr& part, objectives) {
    part->addToModel(m_model);
    objective += part->m_objective;
  }
  m_model->setObjective(objective);
  m_model->update();
}

void Optimizer::clearConvexProblem(const vector<ConvexObjectivePtr>& objectives, const vector<
    ConvexConstraintPtr>& constraints) {
  BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
    part->removeFromModel();
  BOOST_FOREACH(const ConvexPartPtr& part, objectives)
    part->removeFromModel();	
}

Optimizer::OptStatus Optimizer::optimize() {

  ////////////
  double trueImprove, approxImprove, improveRatio; // will be used to check for convergence
  vector<double> newObjectiveVals, objectiveVals, newConstraintVals, constraintVals;
	bool grbFail=false;
  /////////


  for (int iter = 1;;++iter) {

    LOG_INFO_FMT("iteration: %i", iter);

    ////// convexification /////////
    vector<ConvexObjectivePtr> objectives = convexifyObjectives();
    vector<ConvexConstraintPtr> constraints = convexifyConstraints();

    // get objective/constraint values
    // slight optimization: don't evaluate objectives
    // if you just did so while checking for improvement
    if (newObjectiveVals.empty()) objectiveVals = evaluateObjectives(m_costs);
    else objectiveVals = newObjectiveVals;
    if (newConstraintVals.empty()) constraintVals = evaluateConstraints(m_cnts);
    else constraintVals = newConstraintVals;
    double meritVal = sum(objectiveVals) + sum(constraintVals);
        
    // update model with new variables
    m_model->update();
    
    // add non-trust-region stuff to model
    BOOST_FOREACH(const ConvexConstraintPtr& part, constraints)
      part->addToModel(m_model);
    BOOST_FOREACH(const ConvexObjectivePtr& part, objectives) {
      part->addToModel(m_model);
    }

    m_model->update();
        
    ///////////////////////////////////

    int trIter=0;
    while (m_tra->m_shrinkage >= SQPConfig::shrinkLimit) { // trust region adjustment
    	++trIter;
    	LOG_DEBUG_FMT("convex optimization %i", trIter);
      // add trust region stuff
      ConvexObjectivePtr trObjective = m_tra->convexObjective(m_model);
      ConvexConstraintPtr trConstraint = m_tra->convexConstraint(m_model);
      trObjective->addToModel(m_model);
      trConstraint->addToModel(m_model);

      // build objective
      GRBQuadExpr objective(0);
      BOOST_FOREACH(const ConvexObjectivePtr& part, objectives) {
        objective += part->m_objective;
      }      
      objective += trObjective->m_objective;
      m_model->setObjective(objective);

      ////// convex optimization /////////////////////
      preOptimize();
      int grbStatus = convexOptimize();
      if (grbStatus != GRB_OPTIMAL) {
        LOG_ERROR_FMT("bad GRB status: %s. problem written to /tmp/sqp_fail.lp", getGRBStatusString(grbStatus));
	      m_model->write("/tmp/sqp_fail.lp");
				grbFail = true;
				break;
      }
      // updateValues might add/delete variables so we get the approx stuff now
      vector<double> approxObjectiveVals = evalApproxObjectives(objectives);
      double approxMeritVal = sum(approxObjectiveVals);

      storeValues();
      updateValues();
      postOptimize();
      //////////////////////////////////////////////

      ////////// calculate new objectives ////////////
      newObjectiveVals = evaluateObjectives(m_costs);
      newConstraintVals = evaluateConstraints(m_cnts);
      double newMeritVal = sum(newObjectiveVals) + sum(newConstraintVals);
      printObjectiveInfo(objectiveVals, approxObjectiveVals, newObjectiveVals);
      printConstraintInfo(constraintVals, newConstraintVals);
      trueImprove = meritVal - newMeritVal;
      approxImprove = meritVal - approxMeritVal;
      improveRatio = trueImprove / approxImprove;
      LOG_INFO_FMT("%15s | %10.3e | %10.3e | %10.3e | %10.3e", "TOTAL", meritVal, approxImprove, trueImprove, improveRatio);
      //////////////////////////////////////////////

      trObjective->removeFromModel();
      trConstraint->removeFromModel();

      if (approxImprove < 1e-7) {
        LOG_INFO_FMT("not much room to improve in this problem. approxImprove: %.2e", approxImprove);
        if (approxImprove < 0) rollbackValues();
        break;
      }


      if (newMeritVal > meritVal) {
        LOG_INFO("objective got worse! rolling back and shrinking trust region");
        rollbackValues();//
        m_tra->adjustTrustRegion(SQPConfig::trShrink);
      }
      else {
        if (improveRatio < SQPConfig::trThresh)
          m_tra->adjustTrustRegion(SQPConfig::trShrink);
        else
          m_tra->adjustTrustRegion(SQPConfig::trExpand);
        break;
      }

    }
		
		clearConvexProblem(objectives, constraints);

    //// exit conditions ///
    // todo: think through these more carefully
		if (grbFail) {
			return GRB_FAIL;
		}		
    else if (iter >= SQPConfig::maxIter) {
      LOG_WARN("reached iteration limit");
      return ITERATION_LIMIT;
    }
    else if (m_tra->m_shrinkage < SQPConfig::shrinkLimit) {
      LOG_WARN("trust region shrunk too much. stopping");
      return SHRINKAGE_LIMIT;
    }
    else if (approxImprove < 0) {
      LOG_ERROR("approxImprove < 0. something's probably wrong");
      return CONVERGED;
    }
    else if (approxImprove < 1e-7) {
      LOG_INFO("no room to improve, according to convexification");
      return CONVERGED;
    }
    else if (trueImprove < SQPConfig::doneIterThresh && improveRatio > SQPConfig::trThresh) {
      LOG_INFO_FMT("cost improvement below convergence threshold (%.3e < %.3e). stopping", SQPConfig::doneIterThresh, SQPConfig::trThresh);
      return CONVERGED; // xxx should probably check that it improved multiple times in a row
    }

    ///////////////////////

  }
}

#include "expr_ops.h"
void addNormCost(ConvexObjectivePtr& cost, double coeff, const ExprVector& err, GRBModel* model, const string& desc) {
    GRBVar errcost = model->addVar(0,GRB_INFINITY,0, GRB_CONTINUOUS, desc+"_cost");
    cost->m_vars.push_back(errcost);
    GRBQuadExpr qexpr;
    VarVector erris(err.size());
    for (int i=0; i < err.size(); ++i) {
      GRBVar erri = erris[i] = model->addVar(-GRB_INFINITY,GRB_INFINITY,0,GRB_CONTINUOUS, (boost::format("%s_cone_%i")%desc%i).str());
      cost->m_vars.push_back(erri);
      cost->m_eqexprs.push_back(erri-err[i]);
      cost->m_eqcntNames.push_back("cone_coord");
    }
    cost->m_qexprs.push_back(varNorm2(erris) - errcost*errcost);
    cost->m_qcntNames.push_back(desc);
    cost->m_objective += coeff * errcost;
}
void addHingeCost(ConvexObjectivePtr& cost, double coeff, const GRBLinExpr& err, GRBModel* model, const string& desc) {
    GRBVar errcost = model->addVar(0,GRB_INFINITY,0, GRB_CONTINUOUS, desc+"_cost");
    cost->m_vars.push_back(errcost);
    cost->m_exprs.push_back(err - errcost);
    cost->m_cntNames.push_back(desc);
    cost->m_objective += coeff * errcost;
}
void addAbsCost2(ConvexObjectivePtr& cost, double coeff, const GRBLinExpr& err, GRBModel* model, const string& desc) {
    GRBVar errvar = model->addVar(-GRB_INFINITY,GRB_INFINITY,0, GRB_CONTINUOUS, desc);
    cost->m_vars.push_back(errvar);
    GRBVar errcost = model->addVar(0,GRB_INFINITY,0, GRB_CONTINUOUS, desc+"_cost");
    cost->m_vars.push_back(errcost);
    cost->m_eqexprs.push_back(err - errvar);
    cost->m_eqcntNames.push_back(desc);
    cost->m_exprs.push_back(errvar - errcost);
    cost->m_cntNames.push_back(desc+"_cost");
    cost->m_exprs.push_back(- errvar - errcost);
    cost->m_cntNames.push_back(desc+"_cost");
    cost->m_objective += coeff * errcost;
}

void addAbsCost(ConvexObjectivePtr& cost, double coeff, const GRBLinExpr& err, GRBModel* model, const string& desc) {
    GRBVar errcost = model->addVar(0,GRB_INFINITY,0, GRB_CONTINUOUS);
    cost->m_vars.push_back(errcost);
    cost->m_exprs.push_back(err - errcost); /* errcost >= err */
    cost->m_cntNames.push_back(desc); 
    cost->m_exprs.push_back(-err - errcost); /* errcost >= -err */
    cost->m_cntNames.push_back(desc); 
    cost->m_objective += coeff * errcost;
}


