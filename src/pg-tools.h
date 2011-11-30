#ifndef __SOT_PATTERN_GENERATOR_TOOLS_H__
#define __SOT_PATTERN_GENERATOR_TOOLS_H__

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <sot/core/matrix-rotation.hh>

namespace dynamicgraph {
 namespace sot {
  namespace detail
  {
	// convert a matrix4d into a homogeneous matrix
	inline MatrixHomogeneous matrix4dToHomo (const matrix4d & in);

	// compute the static transformation between the inner Joint and the outer joint
	void computeStaticTransformation(MatrixHomogeneous & res,
			const matrix4d& innerJoint, const matrix4d& outerJoint);

	// compute the static transformation between the given joint and the next one.
	// return true if the joint has a toe (<=> is not the last of the kinematic chain)
	bool computeStaticTransformation(MatrixHomogeneous& res, const CjrlJoint* ankle);

	// compute the depth of the zmp in the toe zone (along the foot -> toe axis)
	// if it is negative, the zmp is under the ankle or behind
	// otherwise, it is under the toe (or in front of it)
	double computeToeOverlap( const ml::Vector & zmpPos,
		const MatrixHomogeneous & anklePosition,
		const MatrixHomogeneous & ankle2Toe);
  }
 }
}

#endif // __SOT_PATTERN_GENERATOR_TOES_H__
