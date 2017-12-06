#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

#include "Assignment.h"

using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

bool Assignment::Initialise()
{
	int giraffeSize = 5;
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	dynamicsWorld->setGravity(btVector3(0, -30, 0));
	shared_ptr<PhysicsController> gg = CreateGiraffe(glm::vec3(10, giraffeSize * 7.7, 0), giraffeSize);
	rightShoulderForward = false;

	if (!Game::Initialise()) {
		return false;
	}

	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	if (keyState[SDL_SCANCODE_G])
	{

		if (bodyRightFrontShoulderConstraint->getHingeAngle() >= (shoulderHighLimit * 1.1f) && !rightShoulderForward) {
			rightShoulderForward = true;
		}
		if (bodyRightFrontShoulderConstraint->getHingeAngle() <= (shoulderLowLimit * 0.9f) && rightShoulderForward) {
			rightShoulderForward = false;
		}
		if (bodyRightBackShoulderConstraint->getHingeAngle() >= (shoulderHighLimit*1.1f) && !rightShoulderBack) {
			rightShoulderBack = true;
		}
		if (bodyRightBackShoulderConstraint->getHingeAngle() <= (shoulderLowLimit * 0.9f) && rightShoulderBack) {
			rightShoulderBack = false;
		}



		if (rightShoulderForward) {
			bodyRightFrontShoulderConstraint->enableAngularMotor(true, -250 * timeDelta, 500);
			rightFrontShoulderLegConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
			rightFrontLegHoofConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
		}
		if (!rightShoulderForward) {
			bodyRightFrontShoulderConstraint->enableAngularMotor(true, 250 * timeDelta, 500);
			rightFrontShoulderLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			rightFrontLegHoofConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
		}

		if (rightShoulderForward) {
			bodyLeftFrontShoulderConstraint->enableAngularMotor(true, 250 * timeDelta, 500);
			leftFrontShoulderLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			leftFrontLegHoofConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
		}
		if (!rightShoulderForward) {
			bodyLeftFrontShoulderConstraint->enableAngularMotor(true, -250 * timeDelta, 500);
			leftFrontShoulderLegConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
			leftFrontLegHoofConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
		}
		if (rightShoulderBack) {
			bodyRightBackShoulderConstraint->enableAngularMotor(true, -250 * timeDelta, 500);
			rightBackShoulderTopLegConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
			rightBackBottomLegConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
			rightBackBottomLegHoofConstraint->enableAngularMotor(true, -125 * timeDelta, 500);
		}

		if (!backRightShoulderForward) {
			bodyRightBackShoulderConstraint->enableAngularMotor(true, 250 * timeDelta, 500);
			rightBackShoulderTopLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			rightBackBottomLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			rightBackBottomLegHoofConstraint->enableAngularMotor(true, 125 * timeDelta, 500);
		}
		if (backRightShoulderForward) {
			bodyLeftBackShoulderConstraint->enableAngularMotor(true, 250 * timeDelta, 500);
			leftBackShoulderTopLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			leftBackBottomLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			leftBackBottomLegHoofConstraint->enableAngularMotor(true, 125 * timeDelta, 500);
		}

		if (!backRightShoulderForward) {
			bodyLeftBackShoulderConstraint->enableAngularMotor(true, -250 * timeDelta, 500);
			leftBackShoulderTopLegConstraint->enableAngularMotor(true, -500 * timeDelta, 500);
			leftBackBottomLegConstraint->enableAngularMotor(true, 500 * timeDelta, 500);
			leftBackBottomLegHoofConstraint->enableAngularMotor(true, -125 * timeDelta, 500);
		}
	}

	else {
		bodyLeftFrontShoulderConstraint->enableAngularMotor(true, 250, 500);
		leftFrontShoulderLegConstraint->enableAngularMotor(true, 250, 500);
		leftFrontLegHoofConstraint->enableAngularMotor(true, 250, 500);
		bodyRightFrontShoulderConstraint->enableAngularMotor(true, 250, 500);
		rightFrontShoulderLegConstraint->enableAngularMotor(true, 250, 500);
		rightFrontLegHoofConstraint->enableAngularMotor(true, 250, 500);
		bodyLeftBackShoulderConstraint->enableAngularMotor(true, -250, 500);
		leftBackShoulderTopLegConstraint->enableAngularMotor(true, -250, 500);
		leftBackBottomLegConstraint->enableAngularMotor(true, 250, 500);
		leftBackBottomLegHoofConstraint->enableAngularMotor(true, 250, 500);
		bodyRightBackShoulderConstraint->enableAngularMotor(true, -250, 500);
		rightBackShoulderTopLegConstraint->enableAngularMotor(true, -250, 500);
		rightBackBottomLegConstraint->enableAngularMotor(true, 250, 500);
		rightBackBottomLegHoofConstraint->enableAngularMotor(true, 250, 500);
	}
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> Assignment::CreateGiraffe(glm::vec3 position, float scale) {
	//measurements for creating the rigid bodies for the legs 
	float shoulderRadius = scale / 4;
	float shoulderLength = scale * 2;
	float legRadius = scale / 6;
	float frontLegLength = scale * 3;
	float backLegLength = scale * 1.3;
	float hoofLength = scale / 3;

	//hinge measurements for legs 
	shoulderHighLimit = -glm::radians(75.0f);
	shoulderLowLimit = -glm::radians(150.0f);
	frontLegHighLimit = -glm::radians(40.0f);
	frontLegLowLimit = -glm::radians(180.0f);
	backTopLegHighLimit = glm::radians(15.0f);
	backTopLegLowLimit = glm::radians(45.0f);
	backBottomLegHighLimit = -glm::radians(45.0f);
	backBottomLegLowLimit = -glm::radians(120.0f);
	hoofHighLimit = glm::radians(90.0f);
	hoofLowLimit = -glm::radians(90.0f);
	backHoofHighLimit = glm::radians(170.0f);
	backHoofLowLimit = glm::radians(90.0f);
	backShoulderHighLimit = -glm::radians(75.0f);
	backShoulderLowLimit = -glm::radians(150.0f);

	glm::quat frontShoulderRotation = glm::quat(-0.1, 0, 0, 1);
	glm::quat frontLegRotation = glm::quat(0.3, 0, 0, 1);
	glm::quat hoofRotation = glm::quat(1, 0, 0, 1);
	glm::quat backShoulderRotation = glm::quat(0.1, 0, 0, 1);
	glm::quat backTopLegRotation = glm::quat(-0.5, 0, 0, 1);

	//main body cylinder at position 
	float bodyRadius = scale * 0.55;
	float bodyLength = scale * 3.5;
	glm::quat bodyRotation = glm::quat(1, 0, 0, 1);
	shared_ptr<PhysicsController> body = physicsFactory->CreateCylinder(bodyRadius, bodyLength, position, bodyRotation, false, true);

	//neck and head cylinder
	float headRadius = scale * 0.35;
	float headLength = 4.5;
	float neckRadius = scale * 0.35;
	float neckLength = scale * 4;
	neckHighLimit = glm::radians(115.0f);
	neckLowLimit = glm::radians(115.0f);
	glm::quat neckRotation = glm::quat(1, 0, 0, 1);
	glm::vec3 neckRelativePosition = position + glm::vec3(-bodyLength / 2, -neckLength * 0.5, -bodyRadius / 4);
	glm::vec3 headRelativePosition = neckRelativePosition + glm::vec3(-neckLength / 2 - headRadius * 1.2, bodyRadius, bodyRadius + neckRadius);
	shared_ptr<PhysicsController> neck = physicsFactory->CreateCylinder(neckRadius, neckLength, neckRelativePosition, neckRotation, false, true);
	shared_ptr<PhysicsController> head = physicsFactory->CreateCylinder(headRadius, headLength, headRelativePosition, bodyRotation);

	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -headRadius, 0));
	t2.setOrigin(btVector3(-neckRadius * 1.5, neckLength / 2, 0));
	head_neck = new btFixedConstraint(*head->rigidBody, *neck->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(head_neck);

	neck_body = new btHingeConstraint(*body->rigidBody, *neck->rigidBody, btVector3(bodyLength * 0.5, bodyLength * 0.5, 0), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	neck_body->setLimit(neckLowLimit, neckHighLimit);
	dynamicsWorld->addConstraint(neck_body);

	// right front leg cylinders 
	glm::vec3 rfshoulderRelativePosition = position + glm::vec3(-bodyLength / 2 + bodyLength / 6, -shoulderLength / 2, bodyRadius + shoulderRadius);
	glm::vec3 rflegRelativePosition = rfshoulderRelativePosition + glm::vec3(-shoulderRadius * 2, -shoulderLength * 1.1, 0);
	glm::vec3 rhoofRelativePosition = rflegRelativePosition + glm::vec3(-frontLegLength * 0.6, -frontLegLength * 0.6, 0);
	shared_ptr<PhysicsController> rightFrontShoulder = physicsFactory->CreateCylinder(shoulderRadius, shoulderLength, rfshoulderRelativePosition, frontShoulderRotation, false, true);
	shared_ptr<PhysicsController> rightFrontLeg = physicsFactory->CreateCylinder(legRadius, frontLegLength, rflegRelativePosition, frontLegRotation, false, true);
	shared_ptr<PhysicsController> rightHoof = physicsFactory->CreateCylinder(legRadius, hoofLength, rhoofRelativePosition, hoofRotation, false, true);
	bodyRightFrontShoulderConstraint = new btHingeConstraint(*body->rigidBody, *rightFrontShoulder->rigidBody, btVector3(bodyRadius / 4, bodyLength / 2 - bodyLength / 6, bodyRadius), btVector3(0, -bodyRadius, -shoulderRadius), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	bodyRightFrontShoulderConstraint->setLimit(shoulderLowLimit, shoulderHighLimit);
	Game::dynamicsWorld->addConstraint(bodyRightFrontShoulderConstraint);
	rightFrontShoulderLegConstraint = new btHingeConstraint(*rightFrontShoulder->rigidBody, *rightFrontLeg->rigidBody, btVector3(0, shoulderLength*0.5 + shoulderRadius*0.5, 0), btVector3(0, -frontLegLength*0.5 - legRadius*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	rightFrontShoulderLegConstraint->setLimit(frontLegLowLimit, frontLegHighLimit);
	Game::dynamicsWorld->addConstraint(rightFrontShoulderLegConstraint);
	rightFrontLegHoofConstraint = new btHingeConstraint(*rightFrontLeg->rigidBody, *rightHoof->rigidBody, btVector3(0, frontLegLength*0.5 + legRadius*0.5, 0), btVector3(0, -hoofLength*0.5 - legRadius*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	rightFrontLegHoofConstraint->setLimit(hoofLowLimit, hoofHighLimit);
	Game::dynamicsWorld->addConstraint(rightFrontLegHoofConstraint);

	// left front leg cylinders
	glm::vec3 lfshoulderRelativePosition = position + glm::vec3(-bodyLength / 2 + bodyLength / 6, -shoulderLength / 2, -bodyRadius - shoulderRadius);
	glm::vec3 lflegRelativePosition = lfshoulderRelativePosition + glm::vec3(-shoulderRadius * 2, -shoulderLength * 1.1, 0);
	glm::vec3 lhoofRelativePosition = lflegRelativePosition + glm::vec3(-frontLegLength * 0.6, -frontLegLength * 0.6, 0);
	shared_ptr<PhysicsController> leftFrontShoulder = physicsFactory->CreateCylinder(shoulderRadius, shoulderLength, lfshoulderRelativePosition, frontShoulderRotation, false, true);
	shared_ptr<PhysicsController> leftFrontLeg = physicsFactory->CreateCylinder(legRadius, frontLegLength, lflegRelativePosition, frontLegRotation, false, true);
	shared_ptr<PhysicsController> leftHoof = physicsFactory->CreateCylinder(legRadius, hoofLength, lhoofRelativePosition, hoofRotation, false, true);
	bodyLeftFrontShoulderConstraint = new btHingeConstraint(*body->rigidBody, *leftFrontShoulder->rigidBody, btVector3(bodyRadius / 4, bodyLength / 2 - bodyLength / 6, -bodyRadius), btVector3(0, -bodyRadius / 4, shoulderRadius), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	bodyLeftFrontShoulderConstraint->setLimit(shoulderLowLimit, shoulderHighLimit);
	Game::dynamicsWorld->addConstraint(bodyLeftFrontShoulderConstraint);
	leftFrontShoulderLegConstraint = new btHingeConstraint(*leftFrontShoulder->rigidBody, *leftFrontLeg->rigidBody, btVector3(0, shoulderLength*0.5 + shoulderRadius*0.5, 0), btVector3(0, -frontLegLength*0.5 - legRadius*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	leftFrontShoulderLegConstraint->setLimit(frontLegLowLimit, frontLegHighLimit);
	Game::dynamicsWorld->addConstraint(leftFrontShoulderLegConstraint);
	leftFrontLegHoofConstraint = new btHingeConstraint(*leftFrontLeg->rigidBody, *leftHoof->rigidBody, btVector3(0, frontLegLength*0.5 + legRadius*0.5, 0), btVector3(0, -hoofLength*0.5 - legRadius*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	leftFrontLegHoofConstraint->setLimit(hoofLowLimit, hoofHighLimit);
	Game::dynamicsWorld->addConstraint(leftFrontLegHoofConstraint);

	// right back leg cylinders 
	glm::vec3 rbShoulderRelativePosition = position + glm::vec3(bodyLength / 2 - bodyLength / 6, -shoulderLength / 2, bodyRadius + shoulderRadius);
	glm::vec3 rbtlRelativePosition = rbShoulderRelativePosition + glm::vec3(shoulderRadius*1.5, -shoulderLength / 1.5, 0);
	glm::vec3 rbblRelativePosition = rbtlRelativePosition + glm::vec3(backLegLength * 0.7, -backLegLength, 0);
	glm::vec3 rbhoofRelativePosition = rbblRelativePosition + glm::vec3(-legRadius * 2, -backLegLength, 0);
	shared_ptr<PhysicsController> rightBackShoulder = physicsFactory->CreateCylinder(shoulderRadius, shoulderLength, rbShoulderRelativePosition, backShoulderRotation, false, true);
	shared_ptr<PhysicsController> rightBackTopLeg = physicsFactory->CreateCylinder(legRadius, backLegLength, rbtlRelativePosition, backTopLegRotation, false, true);
	shared_ptr<PhysicsController> rightBackBottomLeg = physicsFactory->CreateCylinder(legRadius, backLegLength, rbblRelativePosition, glm::quat(), false, true);
	shared_ptr<PhysicsController> rightBackHoof = physicsFactory->CreateCylinder(legRadius, hoofLength, rbhoofRelativePosition, hoofRotation, false, true);
	bodyRightBackShoulderConstraint = new btHingeConstraint(*body->rigidBody, *rightBackShoulder->rigidBody, btVector3(bodyRadius / 4, -bodyLength / 2 + bodyLength / 6, bodyRadius), btVector3(0, -bodyRadius / 4, -shoulderRadius), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	bodyRightBackShoulderConstraint->setLimit(backShoulderLowLimit, backShoulderHighLimit);
	Game::dynamicsWorld->addConstraint(bodyRightBackShoulderConstraint);
	rightBackShoulderTopLegConstraint = new btHingeConstraint(*rightBackShoulder->rigidBody, *rightBackTopLeg->rigidBody, btVector3(0, (shoulderLength + shoulderRadius)*0.5, 0), btVector3(0, -(backLegLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	rightBackShoulderTopLegConstraint->setLimit(backTopLegLowLimit, backTopLegHighLimit);
	Game::dynamicsWorld->addConstraint(rightBackShoulderTopLegConstraint);
	rightBackBottomLegConstraint = new btHingeConstraint(*rightBackTopLeg->rigidBody, *rightBackBottomLeg->rigidBody, btVector3(0, (backLegLength + legRadius)*0.5, 0), btVector3(0, -(backLegLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	rightBackBottomLegConstraint->setLimit(backBottomLegLowLimit, backBottomLegHighLimit);
	Game::dynamicsWorld->addConstraint(rightBackBottomLegConstraint);
	rightBackBottomLegHoofConstraint = new btHingeConstraint(*rightBackBottomLeg->rigidBody, *rightBackHoof->rigidBody, btVector3(0, (backLegLength + legRadius)*0.5, 0), btVector3(0, -(hoofLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	rightBackBottomLegHoofConstraint->setLimit(backHoofLowLimit, backHoofHighLimit);
	Game::dynamicsWorld->addConstraint(rightBackBottomLegHoofConstraint);


	//left back leg cylinders 
	glm::vec3 lbShoulderRelativePosition = position + glm::vec3(bodyLength / 2 - bodyLength / 6, -shoulderLength / 2, -bodyRadius - shoulderRadius);
	glm::vec3 lbtlRelativePosition = lbShoulderRelativePosition + glm::vec3(shoulderRadius*1.5, -shoulderLength / 1.5, 0);
	glm::vec3 lbblRelativePosition = lbtlRelativePosition + glm::vec3(backLegLength * 0.7, -backLegLength, 0);
	glm::vec3 lbhoofRelativePosition = lbblRelativePosition + glm::vec3(-legRadius * 2, -backLegLength, 0);
	shared_ptr<PhysicsController> leftBackShoulder = physicsFactory->CreateCylinder(shoulderRadius, shoulderLength, lbShoulderRelativePosition, backShoulderRotation, false, true);
	shared_ptr<PhysicsController> leftBackTopLeg = physicsFactory->CreateCylinder(legRadius, backLegLength, lbtlRelativePosition, backTopLegRotation, false, true);
	shared_ptr<PhysicsController> leftBackBottomLeg = physicsFactory->CreateCylinder(legRadius, backLegLength, lbblRelativePosition, glm::quat(), false, true);
	shared_ptr<PhysicsController> leftBackHoof = physicsFactory->CreateCylinder(legRadius, hoofLength, lbhoofRelativePosition, hoofRotation, false, true);
	bodyLeftBackShoulderConstraint = new btHingeConstraint(*body->rigidBody, *leftBackShoulder->rigidBody, btVector3(bodyRadius / 4, -bodyLength / 2 + bodyLength / 6, -bodyRadius), btVector3(0, -bodyRadius / 4, shoulderRadius), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	bodyLeftBackShoulderConstraint->setLimit(backShoulderLowLimit, backShoulderHighLimit);
	Game::dynamicsWorld->addConstraint(bodyLeftBackShoulderConstraint);
	leftBackShoulderTopLegConstraint = new btHingeConstraint(*leftBackShoulder->rigidBody, *leftBackTopLeg->rigidBody, btVector3(0, (shoulderLength + shoulderRadius)*0.5, 0), btVector3(0, -(backLegLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	leftBackShoulderTopLegConstraint->setLimit(backTopLegLowLimit, backTopLegHighLimit);
	Game::dynamicsWorld->addConstraint(leftBackShoulderTopLegConstraint);
	leftBackBottomLegConstraint = new btHingeConstraint(*leftBackTopLeg->rigidBody, *leftBackBottomLeg->rigidBody, btVector3(0, (backLegLength + legRadius)*0.5, 0), btVector3(0, -(backLegLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	leftBackBottomLegConstraint->setLimit(backBottomLegLowLimit, backBottomLegHighLimit);
	Game::dynamicsWorld->addConstraint(leftBackBottomLegConstraint);
	leftBackBottomLegHoofConstraint = new btHingeConstraint(*leftBackBottomLeg->rigidBody, *leftBackHoof->rigidBody, btVector3(0, (backLegLength + legRadius)*0.5, 0), btVector3(0, -(hoofLength + legRadius)*0.5, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), false);
	leftBackBottomLegHoofConstraint->setLimit(backHoofLowLimit, backHoofHighLimit);
	Game::dynamicsWorld->addConstraint(leftBackBottomLegHoofConstraint);


	// tail cylinder 
	float tailRadius = scale / 6;
	float tailLength = scale;
	glm::quat tailRotation = glm::quat(-0.5, 0, 0, 1);
	glm::vec3 tailStartPosition = position + glm::vec3(bodyLength / 2 + tailLength / 1.2, bodyRadius / 2, 0);
	shared_ptr<PhysicsController> tailStart = physicsFactory->CreateCylinder(tailRadius, tailLength, tailStartPosition, tailRotation, false, true);
	BodyTailConstraint = new btPoint2PointConstraint(*body->rigidBody, *tailStart->rigidBody, btVector3(bodyRadius*0.75, -(bodyLength + tailRadius) / 2, 0), btVector3(0, -tailLength * 0.5, 0));
	Game::dynamicsWorld->addConstraint(BodyTailConstraint);

	return body;
}