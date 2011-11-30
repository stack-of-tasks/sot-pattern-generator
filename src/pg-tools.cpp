#include "pg-tools.h"

namespace dynamicgraph {
 namespace sot {
  namespace detail {

	// convert a matrix4d into a homogeneous matrix
	inline MatrixHomogeneous matrix4dToHomo (const matrix4d & in)
	{
		MatrixHomogeneous out;
		  for (unsigned i=0;i<4; ++i)
			  for (unsigned j=0;j<4; ++j)
				  out(i,j) = MAL_S4x4_MATRIX_ACCESS_I_J(in, i, j);
		return out;
	}


	void computeStaticTransformation(MatrixHomogeneous & res,
			const matrix4d& innerJoint, const matrix4d& outerJoint)
	{
	  MatrixHomogeneous innerJointHomo = matrix4dToHomo(innerJoint);
	  MatrixHomogeneous outerJointHomo = matrix4dToHomo(outerJoint);

	 res = (innerJointHomo.inverse()) * outerJointHomo;
	}


	bool computeStaticTransformation(MatrixHomogeneous& res, const CjrlJoint* ankle)
	{
		int numChildren = ankle->countChildJoints ();
		if (numChildren > 0)
		{
		  const CjrlJoint* ch = ankle->childJoint(0);
		  const matrix4d& ankleInitPos = ankle->initialPosition();
		  const matrix4d& toesInitPos  = ch->initialPosition();

		  // compute the static transformation between ankle and toe.
		  computeStaticTransformation(res,  ankleInitPos, toesInitPos);

		  //TODO (?) Strange patch: force the rotation between the ankle and toe to Identity
		  for (unsigned i=0;i<3; ++i)
			  for (unsigned j=0;j<3; ++j)
				  res(i,j) = ( (i==j)? 1:0);
		  return true;
		}
		else
		{
			return false;
		}
	}


	// compute the position of the zmp in the toe zone
	double computeToeOverlap( const ml::Vector & zmpPos,
		const MatrixHomogeneous & anklePosition,
		const MatrixHomogeneous & ankle2Toe)
	{
		//compute toe position.
		MatrixHomogeneous toeH = anklePosition * ankle2Toe;

		//
		MatrixHomogeneous toeRotation;
		for(unsigned i=0; i<3; ++i)
			for(unsigned j=0; j<3; ++j)
				toeRotation(i,j) = MAL_S4x4_MATRIX_ACCESS_I_J(toeH, j, i);

		ml::Vector zmpPosInToe = zmpPos;
		for(unsigned i=0; i<3; ++i)
			zmpPosInToe(i) -= toeH(i,3);
		zmpPosInToe(2) = 0;

		ml::Vector zmpPosInToeFrame  = toeRotation * zmpPosInToe;

		MatrixHomogeneous toeRotation2;
		for(unsigned i=0; i<3; ++i)
			for(unsigned j=0; j<3; ++j)
				toeRotation(i,j) = MAL_S4x4_MATRIX_ACCESS_I_J(toeH, i, j);
		ml::Vector zmpPosInToeFrame2  = toeRotation2 * zmpPosInToe;

		// if the zmp is in the ankle zone (only the x info...)
		return zmpPosInToeFrame(0) ;
	}
	}
  }
}
