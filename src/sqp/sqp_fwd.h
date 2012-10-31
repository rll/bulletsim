#pragma once
#include <boost/shared_ptr.hpp>
class TrajOptimizer;
typedef boost::shared_ptr<TrajOptimizer> TrajOptimizerPtr;
class TrajComponent;
typedef boost::shared_ptr<TrajComponent> TrajComponentPtr;
class CollisionCost;
typedef boost::shared_ptr<CollisionCost> CollisionCostPtr;
class CartPoseCost;
typedef boost::shared_ptr<CartPoseCost> CartPoseCostPtr;
class JntLenCost;
typedef boost::shared_ptr<JntLenCost> JntLenCostPtr;
class JointBounds;
typedef boost::shared_ptr<JointBounds> JointBoundsPtr;
class CartVelCnt;
typedef boost::shared_ptr<CartVelCnt> CartVelCntPtr;
class CollisionBoxes;
typedef boost::shared_ptr<CollisionBoxes> CollisionBoxesPtr;
class BulletRaveSyncher;
typedef boost::shared_ptr<BulletRaveSyncher> BulletRaveSyncherPtr;
class ArmPrinter;
typedef boost::shared_ptr<ArmPrinter> ArmPrinterPtr;
class PlotHandles;
typedef boost::shared_ptr<PlotHandles> PlotHandlesPtr;
class TrajPlotter;
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;
class TrajChangePlotter;
typedef boost::shared_ptr<TrajChangePlotter> TrajChangePlotterPtr;
class GripperAxesPlotter;
typedef boost::shared_ptr<GripperAxesPlotter> GripperAxesPlotterPtr;
class TwoTrajPlotters;
typedef boost::shared_ptr<TwoTrajPlotters> TwoTrajPlottersPtr;
class GripperPlotter;
typedef boost::shared_ptr<GripperPlotter> GripperPlotterPtr;
class StatePlotter;
typedef boost::shared_ptr<StatePlotter> StatePlotterPtr;
class ArmPlotter;
typedef boost::shared_ptr<ArmPlotter> ArmPlotterPtr;
class TrajPlotter;
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;
class TrajDynSolver;
typedef boost::shared_ptr<TrajDynSolver> TrajDynSolverPtr;
class TrajDynComponent;
typedef boost::shared_ptr<TrajDynComponent> TrajDynComponentPtr;
class TrajDynErrCost;
typedef boost::shared_ptr<TrajDynErrCost> TrajDynErrCostPtr;
class TrajOverlapCost;
typedef boost::shared_ptr<TrajOverlapCost> TrajOverlapCostPtr;
class DynSolver;
typedef boost::shared_ptr<DynSolver> DynSolverPtr;
class DynComponent;
typedef boost::shared_ptr<DynComponent> DynComponentPtr;
class DynErrCost;
typedef boost::shared_ptr<DynErrCost> DynErrCostPtr;
class VelCost;
typedef boost::shared_ptr<VelCost> VelCostPtr;
class DynOverlapCost;
typedef boost::shared_ptr<DynOverlapCost> DynOverlapCostPtr;
class DynTrustRegion;
typedef boost::shared_ptr<DynTrustRegion> DynTrustRegionPtr;
class PushCollision;
typedef boost::shared_ptr<PushCollision> PushCollisionPtr;
class ConvexPart;
typedef boost::shared_ptr<ConvexPart> ConvexPartPtr;
class ConvexObjective;
typedef boost::shared_ptr<ConvexObjective> ConvexObjectivePtr;
class ConvexConstraint;
typedef boost::shared_ptr<ConvexConstraint> ConvexConstraintPtr;
class Cost;
typedef boost::shared_ptr<Cost> CostPtr;
class Constraint;
typedef boost::shared_ptr<Constraint> ConstraintPtr;
class TrustRegion;
typedef boost::shared_ptr<TrustRegion> TrustRegionPtr;
class Optimizer;
typedef boost::shared_ptr<Optimizer> OptimizerPtr;
class PushObject;
typedef boost::shared_ptr<PushObject> PushObjectPtr;
class Collision;
typedef boost::shared_ptr<Collision> CollisionPtr;
class LinkCollision;
typedef boost::shared_ptr<LinkCollision> LinkCollisionPtr;
class JointCollInfo;
typedef boost::shared_ptr<JointCollInfo> JointCollInfoPtr;
class PlanningProblem;
typedef boost::shared_ptr<PlanningProblem> PlanningProblemPtr;
class ProblemComponent;
typedef boost::shared_ptr<ProblemComponent> ProblemComponentPtr;
class CostFunc;
typedef boost::shared_ptr<CostFunc> CostFuncPtr;
class LinearizedCostFunc;
typedef boost::shared_ptr<LinearizedCostFunc> LinearizedCostFuncPtr;
class CollisionCost;
typedef boost::shared_ptr<CollisionCost> CollisionCostPtr;
class VelScaledCollisionCost;
typedef boost::shared_ptr<VelScaledCollisionCost> VelScaledCollisionCostPtr;
class CollisionConstraint;
typedef boost::shared_ptr<CollisionConstraint> CollisionConstraintPtr;
class TrustRegionAdjuster;
typedef boost::shared_ptr<TrustRegionAdjuster> TrustRegionAdjusterPtr;
class LengthConstraintAndCost;
typedef boost::shared_ptr<LengthConstraintAndCost> LengthConstraintAndCostPtr;
class JointBounds;
typedef boost::shared_ptr<JointBounds> JointBoundsPtr;
class CartesianPoseCost;
typedef boost::shared_ptr<CartesianPoseCost> CartesianPoseCostPtr;
class CartesianPoseConstraint;
typedef boost::shared_ptr<CartesianPoseConstraint> CartesianPoseConstraintPtr;
class CartesianVelConstraint;
typedef boost::shared_ptr<CartesianVelConstraint> CartesianVelConstraintPtr;
class CostFuncEvaluator;
typedef boost::shared_ptr<CostFuncEvaluator> CostFuncEvaluatorPtr;
class CostGradEvaluator;
typedef boost::shared_ptr<CostGradEvaluator> CostGradEvaluatorPtr;
class fScalarOfScalar;
typedef boost::shared_ptr<fScalarOfScalar> fScalarOfScalarPtr;
class fScalarOfVector;
typedef boost::shared_ptr<fScalarOfVector> fScalarOfVectorPtr;
class fScalarOfMatrix;
typedef boost::shared_ptr<fScalarOfMatrix> fScalarOfMatrixPtr;
class fVectorOfScalar;
typedef boost::shared_ptr<fVectorOfScalar> fVectorOfScalarPtr;
class fVectorOfVector;
typedef boost::shared_ptr<fVectorOfVector> fVectorOfVectorPtr;
class fVectorOfMatrix;
typedef boost::shared_ptr<fVectorOfMatrix> fVectorOfMatrixPtr;
class fMatrixOfScalar;
typedef boost::shared_ptr<fMatrixOfScalar> fMatrixOfScalarPtr;
class fMatrixOfVector;
typedef boost::shared_ptr<fMatrixOfVector> fMatrixOfVectorPtr;
class fMatrixOfMatrix;
typedef boost::shared_ptr<fMatrixOfMatrix> fMatrixOfMatrixPtr;
class QuadFuncFromGRB;
typedef boost::shared_ptr<QuadFuncFromGRB> QuadFuncFromGRBPtr;
class StateSetter;
typedef boost::shared_ptr<StateSetter> StateSetterPtr;
class RobotJointSetter;
typedef boost::shared_ptr<RobotJointSetter> RobotJointSetterPtr;
class ComboStateSetter;
typedef boost::shared_ptr<ComboStateSetter> ComboStateSetterPtr;
class ObjectPoseSetter;
typedef boost::shared_ptr<ObjectPoseSetter> ObjectPoseSetterPtr;
class ScopedStateSave;
typedef boost::shared_ptr<ScopedStateSave> ScopedStateSavePtr;
