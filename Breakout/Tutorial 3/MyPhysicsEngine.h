#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	//Square of cloth class
	class clothSquare
	{
	public:
		clothSquare(PxPhysics* physics, Scene* scene, const PxTransform& pose = PxTransform(PxIdentity)) {
			//cloth square vertices
			PxClothParticle cloth_verts[] = { PxClothParticle(PxVec3(0,0,0),0), PxClothParticle(PxVec3(0,0,1),1), PxClothParticle(PxVec3(1,0,0),1), PxClothParticle(PxVec3(1,0,1),1) };
			static PxU32 cloth_trigs[] = { 0,1,4,3,0 };

			//cloth
			PxClothMeshDesc meshDesc;
			meshDesc.points.data = cloth_verts;
			meshDesc.points.count = 4;
			meshDesc.points.stride = sizeof(PxClothParticle);

			meshDesc.invMasses.data = &cloth_verts->invWeight;
			meshDesc.invMasses.count = 4;
			meshDesc.invMasses.stride = sizeof(PxClothParticle);

			meshDesc.quads.data = cloth_verts;
			meshDesc.quads.count = 1;
			meshDesc.quads.stride = sizeof(PxU32) * 4;

			//fabric
			PxClothFabric* fabric = PxClothFabricCreate(*physics, meshDesc, PxVec3(0, -1, 0));

			//create cloth
			PxCloth* cloth = physics->createCloth(pose, *fabric, cloth_verts, PxClothFlags());
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger, reset;
		int curScore = 0;
		PxRigidActor* hitObject;

		MySimulationEventCallback() : trigger(false), reset(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
						reset = true;
						hitObject = pairs[i].triggerActor->is<PxRigidActor>();
						curScore++;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Box* paddle, *leftWall, *rightWall, *topWall;
		Walls* border;
		Brick* brick, *brick1, *brick2, *brick3, *brick4, *brick5, *brick6, *brick7, *brick8, *brick9, *brick10, *brick11, *brick12, *brick13, *brick14, *brick15, *brick16, *brick17, *brick18, *brick19, *brick20, *brick21, *brick22, *brick23, *brick24, *brick25, *brick26, *brick27, *brick28;
		Sphere* ball, *bumper;
		MySimulationEventCallback* my_callback;
		RevoluteJoint* rJoint;
		bool reset;
		int curScore;
		
	public:

		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};
		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.1f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);
			reset = false;

			//initialise score
			curScore = 0;

			//Floor that is background to game
			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);

			//Paddle to hit ball
			//Kinematic to avoid unwanted movement, high restitution to bounce ball
			paddle = new Box(PxTransform(PxVec3(0.f, .15f, -1.f)), PxVec3(.75f,.1f,.1f));
			PxMaterial* myMaterial = CreateMaterial(0.f, 0.1f, .52f);
			paddle->Material(myMaterial);
			paddle->SetKinematic(1);
			paddle->Get()->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, 1);
			Add(paddle);

			//Ball
			//High restitution to bounce, set to ballActor for use in visual debugger
			ball = new Sphere(PxTransform(PxVec3(-4.f, .1f, -4.f)), .1f);
			PxMaterial* myMaterial2 = CreateMaterial(0.f, 0.f, 1.65f);
			ball->Material(myMaterial2);
			ballActor = ball->Get()->is<PxRigidDynamic>();
			Add(ball);

			//Walls that surround game area
			border = new Walls();
			border->Material(myMaterial);
			Add(border);

			//Bricks to form wall target
			brick = new Brick(PxTransform(PxVec3(0.f, 0.15f, -5.f))); Add(brick);
			brick1 = new Brick(PxTransform(PxVec3(0.8f, 0.15f, -5.f))); Add(brick1);
			brick2 = new Brick(PxTransform(PxVec3(1.6f, 0.15f, -5.f))); Add(brick2);
			brick3 = new Brick(PxTransform(PxVec3(2.4f, 0.15f, -5.f))); Add(brick3);
			brick4 = new Brick(PxTransform(PxVec3(3.2f, 0.15f, -5.f))); Add(brick4);
			brick5 = new Brick(PxTransform(PxVec3(-0.8f, 0.15f, -5.f))); Add(brick5);
			brick6 = new Brick(PxTransform(PxVec3(-1.6f, 0.15f, -5.f))); Add(brick6);
			brick7 = new Brick(PxTransform(PxVec3(-2.4f, 0.15f, -5.f))); Add(brick7);
			brick8 = new Brick(PxTransform(PxVec3(-3.2f, 0.15f, -5.f))); Add(brick8);
			brick9 = new Brick(PxTransform(PxVec3(0.f, 0.15f, -5.4f))); Add(brick9);
			brick10 = new Brick(PxTransform(PxVec3(0.8f, 0.15f, -5.4))); Add(brick10);
			brick11 = new Brick(PxTransform(PxVec3(1.6f, 0.15f, -5.4f))); Add(brick11);
			brick12 = new Brick(PxTransform(PxVec3(2.4f, 0.15f, -5.4f))); Add(brick12);
			brick13 = new Brick(PxTransform(PxVec3(3.2, 0.15f, -5.4f))); Add(brick13);
			brick14 = new Brick(PxTransform(PxVec3(-0.8f, 0.15f, -5.4f))); Add(brick14);
			brick15 = new Brick(PxTransform(PxVec3(-1.6f, 0.15f, -5.4f))); Add(brick15);
			brick16 = new Brick(PxTransform(PxVec3(-2.4f, 0.15f, -5.4f))); Add(brick16);
			brick17 = new Brick(PxTransform(PxVec3(-3.2f, 0.15f, -5.4f))); Add(brick17);
			brick18 = new Brick(PxTransform(PxVec3(0.f, 0.15f, -5.8f))); Add(brick18);
			brick19 = new Brick(PxTransform(PxVec3(0.8f, 0.15f, -5.8f))); Add(brick19);
			brick20 = new Brick(PxTransform(PxVec3(1.6f, 0.15f, -5.8f))); Add(brick20);
			brick21 = new Brick(PxTransform(PxVec3(2.4f, 0.15f, -5.8f))); Add(brick21);
			brick22 = new Brick(PxTransform(PxVec3(3.2, 0.15f, -5.8f))); Add(brick22);
			brick23 = new Brick(PxTransform(PxVec3(-0.8f, 0.15f, -5.8f))); Add(brick23);
			brick24 = new Brick(PxTransform(PxVec3(-1.6f, 0.15f, -5.8f))); Add(brick24);
			brick25 = new Brick(PxTransform(PxVec3(-2.4f, 0.15f, -5.8f))); Add(brick25);
			brick26 = new Brick(PxTransform(PxVec3(-3.2f, 0.15f, -5.8f))); Add(brick26);
			
			//Rotating brick
			brick27 = new Brick(PxTransform(PxVec3(4.5f, 0.15f, -8.f))); Add(brick27);
			rJoint = new RevoluteJoint(NULL, PxTransform(PxVec3(4.5f, 0.15f, -8.f), PxQuat(PxPi/2, PxVec3(0.f, 0.f, 1.f))), brick27, PxTransform(PxVec3(0.f,0.f,1.f), PxQuat(PxPi / 2, PxVec3(0.f, 0.f, 1.f))));
			rJoint->DriveVelocity(1.0f);

			//Limit y-axis of paddle
			d6YLimit dJointPaddle(NULL, PxTransform(PxVec3(0.f, 0.f, -1.f)), paddle, PxTransform(PxVec3(0.f, 0.f, 0.f)));

			

		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			//Check for contact with trigger object
			reset = my_callback->reset;
			if (reset) {
				//Move target brick out of game area
				my_callback->hitObject->setGlobalPose(PxTransform(PxVec3(50.f, 1.f, 0.f)));
				curScore = my_callback->curScore;
			}

		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}

		int getScore() {
			return curScore;
		}
	};
}
