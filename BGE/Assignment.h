#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Assignment :
		public Game
	{
	private:

		btHingeConstraint * bodyRightFrontShoulderConstraint;
		btHingeConstraint * rightFrontShoulderTopLegConstraint;
		btHingeConstraint * rightFrontShoulderLegConstraint;
		btHingeConstraint * rightFrontBottomLegConstraint;
		btHingeConstraint * rightFrontLegHoofConstraint;
		btHingeConstraint * bodyRightBackShoulderConstraint;
		btHingeConstraint * rightBackShoulderTopLegConstraint;
		btHingeConstraint * rightBackBottomLegConstraint;
		btHingeConstraint * rightBackBottomLegHoofConstraint;
		btHingeConstraint * bodyLeftFrontShoulderConstraint;
		btHingeConstraint * leftFrontShoulderTopLegConstraint;
		btHingeConstraint * leftFrontShoulderLegConstraint;
		btHingeConstraint * leftFrontBottomLegConstraint;
		btHingeConstraint * leftFrontLegHoofConstraint;
		btHingeConstraint * bodyLeftBackShoulderConstraint;
		btHingeConstraint * leftBackShoulderTopLegConstraint;
		btHingeConstraint * leftBackBottomLegConstraint;
		btHingeConstraint * leftBackBottomLegHoofConstraint;
		btHingeConstraint * neck_body;
		//btFixedConstraint * neck_body;
		btFixedConstraint * head_neck;
		btPoint2PointConstraint * BodyTailConstraint;

		bool rightShoulderForward = false;
		bool rightShoulderBack = false;
		bool backRightShoulderForward = false;
		bool backRightShoulderBack = false;

		int shoulderHighLimit;
		int shoulderLowLimit;
		int neckHighLimit;
		int neckLowLimit;
		int frontLegHighLimit;
		int frontLegLowLimit;
		int backTopLegHighLimit;
		int backTopLegLowLimit;
		int backBottomLegHighLimit;
		int backBottomLegLowLimit;
		int hoofHighLimit;
		int hoofLowLimit;
		int backHoofHighLimit;
		int backHoofLowLimit;
		int backShoulderHighLimit;
		int backShoulderLowLimit;
	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		shared_ptr<PhysicsController> CreateGiraffe(glm::vec3 position, float scale = 5);
	};
}
