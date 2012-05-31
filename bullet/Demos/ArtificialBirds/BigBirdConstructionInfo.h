#ifndef BIG_BIRD_CONSTRUCTION_INFO__
#define BIG_BIRD_CONSTRUCTION_INFO__

struct BigBirdConstructionInfo {
	// Start Offset
	btTransform startTransform;

	// Geometry
	btScalar pelvisHalfLength;
	btScalar wingHalfLength;

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

};

#endif