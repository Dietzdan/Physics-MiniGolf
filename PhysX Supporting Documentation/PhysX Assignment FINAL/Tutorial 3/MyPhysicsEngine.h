#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>
#include <Chrono>
#include <ctime>


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

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

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
						
						if (pairs[i].otherShape->getGeometryType() != PxGeometryType::eBOX)
						{
							trigger = true;
							
						}
						
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
						cerr << "YOU WIN!!!!" << endl;
						
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
					//PxRigidBody* Ball = pairHeader.actors[0]->isRigidBody()->getLinearVelocity();
						
					PxVec3 BallVelocity = pairHeader.actors[0]->isRigidBody()->getLinearVelocity();
				
					
					//Ball->setLinearVelocity(BallVelocity/2);
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
			pairFlags |= PxPairFlag::eMODIFY_CONTACTS;
				
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
			
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane,* plane1;
		Box* Blade, * Blade2, * Windmill, * Base1, * Base2, * ClubController, * Floor, *Tee;
		Sphere* GolfBall;
		vector<Box*> Walls;
		Club* GolfClub;
		Flag* EndFlag;
		//materials
		PxMaterial* Sand = CreateMaterial(0.9f, 0.9f, 0.9f);
		PxMaterial* Concrete = CreateMaterial(0.6f, 0.6f, 0.6f);
		PxMaterial* GBall = CreateMaterial(0.2f, 0.2f, 0.858f);
		PxMaterial* Grass = CreateMaterial(0.35f, 0.35f, 0.35f);
		PxMaterial* Wood = CreateMaterial(0.25f, 0.25f, 0.25f);
		//Testing Triangle mesh
		ConvexMesh* Ramp;
		ConvexMesh* Wedge1;
		ConvexMesh* Wedge2;
		vector<PxVec3> Vert;
		std::vector<PxU32> Trigs;
		bool onTee;

		//testing
		clock_t current_ticks, delta_ticks;
		clock_t fps = 0;
		
		
		MySimulationEventCallback* my_callback;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS,1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			//GetMaterial()->setDynamicFriction(.5f);
			//GetMaterial()->setDynamicFriction(1.5f);
			
			//Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(255.f,210.f/255.f,255.f/255.f));
			plane->Material(Grass);
			Add(plane);
			
			
				GolfBall = new Sphere(PxTransform(PxVec3(3.0f, 1.5f, 8.f)));
				GolfBall->Color(PxVec3(245.0f / 225.f, 245.0f / 255.f, 245.0f / 255.f));
				GolfBall->GetShape()->setGeometry(PxSphereGeometry(0.5f));
				GolfBall->Name("Ball");
				GolfBall->Material(GBall);
				Add(GolfBall);
			
			
			//Club
			GolfClub = new Club(PxTransform(PxVec3(-3.0f, 2.0f, 3.f)));
			GolfClub->Name("Club");
			Add(GolfClub);
			ClubController = new Box(PxTransform(PxVec3(3.0f, 11.6f, 8.f))); // y=10.4 for ground y=11.4 for tee
			ClubController->Name("ClubController");
			ClubController->GetShape()->setGeometry(PxBoxGeometry(.5f, .5f, .5f));
			ClubController->SetKinematic(true);
			Add(ClubController);
			RevoluteJoint joint(ClubController, PxTransform(PxVec3(0.f,0.f,0.f),PxQuat(PxPi/2,PxVec3(0.f,1.f,0.f))), GolfClub, PxTransform(PxVec3(0.f,10.f,0.f)));
			
			//Tee
			Tee = new Box(PxTransform(PxVec3(3.0f, 0.0f, 8.f)));
			Tee->GetShape()->setGeometry(PxBoxGeometry(1.0f, 1.0f, 1.0f));
			Tee->Name("Tee");
			Tee->Material(Concrete);
			Add(Tee);

			//Flag 
			EndFlag = new Flag(PxTransform(PxVec3(300.0f, 1.5f, 2.0f)));
			EndFlag->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			EndFlag->Name("Flag");
			EndFlag->Color(PxVec3(255.0f / 255.0f, 10.0f / 255.0f, 10.0f / 255.0f));
			Add(EndFlag);

			//Floors Sand
			Floor = new Box(PxTransform(PxVec3(40.f, -0.4f, 7.5f)));
			Floor->Name("SandFloor");
			Floor->Material(Sand);
			Floor->Color(PxVec3(225.f / 225.f, 225.f / 255.f, 0.f));
			Floor->GetShape()->setGeometry(PxBoxGeometry(16.f, .5f, 7.f));
			Add(Floor);

			//Floor Grass
			Floor = new Box(PxTransform(PxVec3(88.f, -0.4f, 7.5f)));
			Floor->Name("GrassFloor");
			Floor->Material(Grass);
			Floor->Color(PxVec3(10.f / 225.f, 245.f / 255.f, 10.f / 255.f));
			Floor->GetShape()->setGeometry(PxBoxGeometry(32.f, .5f, 7.f));
			Add(Floor);

			//Floor Concrete
			Floor = new Box(PxTransform(PxVec3(184.f, -0.4f, 7.5f)));
			Floor->Name("ConcreteFloor");
			Floor->Material(Concrete);
			Floor->Color(PxVec3(211.f/255.f, 211.f / 255.f, 211.f / 255.f));
			Floor->GetShape()->setGeometry(PxBoxGeometry(64.f, .5f, 7.f));
			Add(Floor);
			//side walls
			for (int i = 2; i < 61; i++)
			{
				float z = .0f;
				if (i % 2 == 0 )
				{
					z += 15.0;
				}
				
				Box* Wall = new Box(PxTransform(PxVec3(11.f*(i/2), 1.5f, z)));
				Wall->Name("Wall");
				Wall->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, 0.0f, .0f), PxQuat(1.55f, PxVec3(1.0f, .0f, .0f))));
				Wall->Color(color_palette[0]);
				Wall->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
				Wall->Material(Wood);
				Add(Wall);
				Walls.push_back(Wall);
			}

			//end walls
			Box* EndWall1 = new Box(PxTransform(PxVec3(307.0f, 3.5f, 8.f),PxQuat(1.55f, PxVec3(.0f, 1.0f, .0f))));
			EndWall1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f)));//, PxQuat(1.5f, PxVec3(0.0f, 0.0f, 1.0f))));
			EndWall1->GetShape()->setGeometry(PxBoxGeometry(7.5f, .5f, 3.5f));
			EndWall1->Name("Wall");
			Box* EndWall2 = new Box(PxTransform(PxVec3(310.0f, 1.5f, 8.f), PxQuat(1.55f, PxVec3(1.0f, .0f, .0f))));
			EndWall2->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(1.55f, PxVec3(0.0f, 0.0f, 1.0f))));
			EndWall2->GetShape()->setGeometry(PxBoxGeometry(7.5f, .5f, 1.5f));
			EndWall2->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			EndWall2->GetShape()->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
			EndWall2->Name("GameoverWall");
			Add(EndWall1);
			Add(EndWall2);
			
			//Creating the Ramp
			
				vector<PxVec3> Ramp2({ PxVec3(4,1,0),PxVec3(4,0,0),PxVec3(0,0,2),PxVec3(0,0,-2) });
				Ramp = new ConvexMesh(Ramp2, PxTransform(PxVec3(20.0f, 0.0f, 8.f)));

				Ramp->Color(color_palette[0]);
				Ramp->Name("Ramp");
				Ramp->Material(Concrete);
				Add(Ramp);
			
		
				//WindMill
				Blade = new Box(PxTransform(PxVec3(159.f, 7.f, 8.f), PxQuat(1.55f, (PxVec3(.0f, 1.f, .0f)))));
				Blade->GetShape()->setGeometry(PxBoxGeometry(7.f, 0.5f, 0.1f));
				Blade->Material(Wood);

				Blade2 = new Box(PxTransform(PxVec3(159.f, 7.f, 8.f), PxQuat(1.5f, (PxVec3(.0f, 1.f, .0f)))));
				Blade2->GetShape()->setGeometry(PxBoxGeometry(0.5f, 7.f, 0.1f));
				Blade2->Material(Wood);


				Windmill = new Box(PxTransform(PxVec3(160.f, 5.f, 8.f), PxQuat(1.55f, (PxVec3(.0f, 1.f, .0f)))));
				Windmill->GetShape()->setGeometry(PxBoxGeometry(2.f, 3.0f, .8f));
				Windmill->Material(Wood);
				Windmill->Color(PxVec3(160.f / 255.f, 82.f / 255.f, 45.f / 255.f));
				Base1 = new Box(PxTransform(PxVec3(160.f, 0.f, 6.5f), PxQuat(1.55f, (PxVec3(.0f, 1.f, .0f)))));
				Base1->GetShape()->setGeometry(PxBoxGeometry(0.5f, 2.f, .8f));
				Base1->Material(Wood);
				Base1->Color(PxVec3(160.f / 255.f, 82.f / 255.f, 45.f / 255.f));
				Base2 = new Box(PxTransform(PxVec3(160.f, 0.f, 9.5f), PxQuat(1.55f, (PxVec3(.0f, 1.f, .0f)))));
				Base2->GetShape()->setGeometry(PxBoxGeometry(0.5f, 2.f, .8f));
				Base2->Material(Wood);
				Base2->Color(PxVec3(160.f / 255.f, 82.f / 255.f, 45.f / 255.f));

			
			
			//Creating the wedge obstacles
			vector<PxVec3> WedgeVerts({ PxVec3(6.5f,2,0),PxVec3(6.5f,0,0),PxVec3(0,2,6.5f),PxVec3(0,0,-6.5f),PxVec3(0,0,6.5f),PxVec3(0,2,-6.5f)});
			Wedge1 = new ConvexMesh(WedgeVerts, PxTransform(PxVec3(90.0f, 0.0f, 14.5f)));
			Wedge2 = new ConvexMesh(WedgeVerts, PxTransform(PxVec3(110.0f, 0.0f, 0.5f)));
			Wedge1->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(1.5f, PxVec3(.0f, 1.0f, .0f))));
			Wedge1->Color(color_palette[3]);
			Wedge2->GetShape()->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(-1.5f, PxVec3(.0f, 1.0f, .0f))));
			Wedge1->Name("Wedge1");
			Wedge2->Name("Wedge2");
			Wedge1->Material(Wood);
			Wedge2->Material(Wood);
			Add(Wedge1);
			Add(Wedge2);
			

			//Wedge2->GetShape()->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
			//Wedge2->GetShape()->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);
			//set collision filter flags
			GolfBall->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			//use | operator to combine more actors e.g.
			 GolfBall->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			//don't forget to set your flags for the matching actor as well, e.g.:
			Wedge1->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			Wedge2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);

			//box->Name("Box1");
			//box2->Name("Box2");
			
			//add blades last so if statement works properly
			Add(Windmill);
			Add(Base1);
			Add(Base2);
			Add(Blade);
			Add(Blade2);
			
			
			
			// setting the mass of every object to 0 unless its the club or the ball 
			vector<PxRigidDynamic*> actors(px_scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC));
			if (actors.size() && (px_scene->getActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC, (PxActor**)&actors.front(), (PxU32)actors.size())))
			{
				actors[0]->setLinearDamping(0.f);
				actors[0]->setMass(3.f);
				actors[0]->setMassSpaceInertiaTensor(PxVec3(0.f));
				actors[1]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
				
				//2 because 0 is the ball and club is 1
				for (int i = 2; i < actors.size(); i++)
				{	
					actors[i]->setMass(0.f);
					actors[i]->setMassSpaceInertiaTensor(PxVec3(0.f));
					actors[i]->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
					if (i == actors.size() - 1 || i == actors.size()-2)
					{
						actors[i]->setAngularVelocity(PxVec3(2.f, 0.f, 0.f));
						actors[i]->setAngularDamping(0.f);
					 }

				}
			}
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			
			////cerr << GolfClub->Get()->isRigidBody()->getGlobalPose().p.x << " " << GolfClub->Get()->isRigidBody()->getGlobalPose().p.y << " " << GolfClub->Get()->isRigidBody()->getGlobalPose().p.z << endl;
			PxTransform pose = ((PxRigidBody*)GolfBall->Get())->getGlobalPose();
			PxTransform ClubPose = ((PxRigidBody*)GolfClub->Get())->getGlobalPose();
			PxTransform ControllerPos = ((PxRigidBody*)ClubController->Get())->getGlobalPose();
			PxVec3 ClubVelocity = ((PxRigidBody*)GolfClub->Get())->getAngularVelocity();
			////cerr << ClubVelocity.x << " " << ClubVelocity.y << " " << ClubVelocity.z << endl;
			PxVec3 BallPos = pose.p;
			if (BallPos.x == 3.0f)
			{	
				onTee = true;
			}
			else
			{
				onTee = false;
			}

			
			
			//check if ball is on tee for club height
			if (onTee == false)
			{
				ClubController->Get()->isRigidBody()->setGlobalPose(PxTransform(PxVec3(BallPos.x, 10.4f, BallPos.z)));
				
			}
			if (onTee == true)
			{
				ClubController->Get()->isRigidBody()->setGlobalPose(PxTransform(PxVec3(BallPos.x, 11.7f, BallPos.z)));
			}
		
			//player feed backl and restart the game
			if (my_callback->trigger == true)
			{
				EndFlag->Color(PxVec3(10.f / 225.f, 245.f / 255.f, 10.f / 255.f));
				GolfBall->Get()->isRigidBody()->setGlobalPose(PxTransform(PxVec3(3.0f, 1.f, 8.0f)));
				GolfBall->Get()->isRigidBody()->setLinearVelocity(PxVec3(0.0f, 0.0f, 0.0f));
				
			}

			
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			
		}

	
	};
}
