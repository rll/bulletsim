#include <boost/shared_ptr.hpp>
class CollisionBoxes;
typedef boost::shared_ptr<CollisionBoxes> CollisionBoxesPtr;
class PlanningProblem;
typedef boost::shared_ptr<PlanningProblem> PlanningProblemPtr;
class ProblemComponent;
typedef boost::shared_ptr<ProblemComponent> ProblemComponentPtr;
class CollisionCost;
typedef boost::shared_ptr<CollisionCost> CollisionCostPtr;
class SimpleCollisionCost;
typedef boost::shared_ptr<SimpleCollisionCost> SimpleCollisionCostPtr;
class VelScaledCollisionCost;
typedef boost::shared_ptr<VelScaledCollisionCost> VelScaledCollisionCostPtr;
class CollisionConstraint;
typedef boost::shared_ptr<CollisionConstraint> CollisionConstraintPtr;
class LengthConstraintAndCost;
typedef boost::shared_ptr<LengthConstraintAndCost> LengthConstraintAndCostPtr;
class JointBounds;
typedef boost::shared_ptr<JointBounds> JointBoundsPtr;
class CartesianPoseCost;
typedef boost::shared_ptr<CartesianPoseCost> CartesianPoseCostPtr;
class TrajPlotter;
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;
class GripperPlotter;
typedef boost::shared_ptr<GripperPlotter> GripperPlotterPtr;
class ArmPlotter;
typedef boost::shared_ptr<ArmPlotter> ArmPlotterPtr;
class ScalarValuedFunction;
typedef boost::shared_ptr<ScalarValuedFunction> ScalarValuedFunctionPtr;
class Collision;
typedef boost::shared_ptr<Collision> CollisionPtr;
class BulletRaveSyncher;
typedef boost::shared_ptr<BulletRaveSyncher> BulletRaveSyncherPtr;
class LinkCollision;
typedef boost::shared_ptr<LinkCollision> LinkCollisionPtr;
class JointCollInfo;
typedef boost::shared_ptr<JointCollInfo> JointCollInfoPtr;
class CollisionCostEvaluator;
typedef boost::shared_ptr<CollisionCostEvaluator> CollisionCostEvaluatorPtr;
class GenericCCE;
typedef boost::shared_ptr<GenericCCE> GenericCCEPtr;
class ArmCCE;
typedef boost::shared_ptr<ArmCCE> ArmCCEPtr;
