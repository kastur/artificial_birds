#ifndef BIG_BIRD_CONSTRUCTION_INFO__
#define BIG_BIRD_CONSTRUCTION_INFO__

struct BigBirdConstructionInfo {

	int birdId;
	
	//Hoist offset
	btTransform hoistTransform;

	// Start Offset
	btTransform startTransform;

	// Geometry
	btScalar pelvisHalfLength;
	btScalar wingHalfLength;

	btScalar hoistAngleXY;
	btScalar hoistAngleZXY;
	btScalar hoistMass;
	btScalar pelvisMass;
	btScalar wingMass;

	btScalar featherArea;  // unused.
	btVector3 pelvisRelPosToAttachWing;
	btVector3 featherRelPosToAttachFeather;

	// Wingbeat
	btScalar wingFlapMotorMaxImpulse;
	btScalar wingFlapFrequency;
	btScalar wingFlapHingeLimit;
	btScalar featherAoAMotorMaxImpulse;
	btScalar featherAoAHingeLimit;

	unsigned int randSeed;
	int numPoints;
	btScalar* reqWingFlappingAngle;
	btScalar* reqFeatherAngleOfAttack1;
	btScalar* reqFeatherAngleOfAttack2;
	btScalar* reqFeatherAngleOfAttack3;

};

#endif