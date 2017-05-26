# include <unistd.h>
# include <map>
# include <vector>
# include <iostream>
# include <chrono>

# include "Graphics.hpp"
# include <PxPhysicsAPI.h>


using namespace physx;
using EntityID = int;

//// Globals ////
PxDefaultAllocator			gAllocator;
PxDefaultErrorCallback		gErrorCallback;
PxFoundation*				gFoundation = nullptr;
PxDefaultCpuDispatcher*		gDispatcher = nullptr;
PxCooking*					gCooking = nullptr;
PxPhysics*					gPhysics = nullptr;
PxMaterial*					gPhysicsMaterial = nullptr;
PxScene* 					gPhysicsScene = nullptr;

const vec3 VEC3_ZERO = vec3(0.f, 0.f, 0.f);


//// Structs ////
struct Entity
{
	vec3 			position 	= vec3(1.f, 1.f, 1.f);
	quat 			rotation 	= quat(0.f, 0.f, 0.f, 1.f);
	vec3 			scale 		= vec3(1.f, 1.f, 1.f);

	mat4 			getModelMatrix( void ) {
		mat4 model = mat4_cast(rotation);
		model = model*glm::scale(mat4(1.f), scale);
		model = glm::translate(mat4(1.f), position)*model;
		return model;
	};
};

struct DynamicEntity : public Entity
{
	using Ptr = std::shared_ptr<DynamicEntity>;

	PxRigidDynamic*	body = nullptr;
};

struct StaticEntity : public Entity
{
	using Ptr = std::shared_ptr<StaticEntity>;

	PxRigidStatic*	body = nullptr;
};


//// Utility Functions ////
inline physx::PxVec3 	toPxVec3( vec3 v ) { return physx::PxVec3(v.x, v.y, v.z); }
inline physx::PxQuat 	toPxQuat( quat q ) { return physx::PxQuat(q.x, q.y, q.z, q.w); }
inline vec3 			toVec3( PxVec3 v ) { return vec3(v.x, v.y, v.z); }
inline quat 			toQuat( PxQuat q ) { return quat(q.w, q.x, q.y, q.z); }

static void 	printBinary( PxU32 word )
{
	for (uint i = 0; i < 32; ++i)
		std::cout << int(((1 << i) & word)? 1 : 0);
}

static void 	debugDisplayFilterData( DynamicEntity& entity )
{
	PxRigidDynamic* dyn = entity.body;

	std::vector<PxShape*> 	shapes(dyn->getNbShapes());
	dyn->getShapes(&shapes[0], shapes.size());

	std::cout << "debugDisplayFilterData"<< std::endl;
	for (PxShape* s : shapes)
	{
		PxFilterData fd = s->getSimulationFilterData();
		std::cout << "FilterData: ";
		printBinary(fd.word0);
		std::cout << ", ";
		printBinary(fd.word1);
		std::cout << ", ";
		printBinary(fd.word2);
		std::cout << ", ";
		printBinary(fd.word3);
		std::cout << std::endl;
	}
}

//// Physics Functions ////
static bool 	initPhysics( void )
{
	if (gFoundation)
		return false; // already init

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = 
		&PxProfileZoneManager::createProfileZoneManager(gFoundation);
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, 
			PxTolerancesScale(),true,profileZoneManager);

	gDispatcher = PxDefaultCpuDispatcherCreate(2);

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, 
			PxCookingParams(gPhysics->getTolerancesScale()));
	//PxCookingParams(toleranceScale));

	if (!gCooking)
		std::cout << "PxCreateCooking failed!" << std::endl;

	gPhysicsMaterial = 
		gPhysics->createMaterial(0.5f, 0.5f, 0.6f); //static friction, dynamic friction, restitution

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gPhysicsScene = gPhysics->createScene(sceneDesc);

	return true;
}

static void 	deinitPhysics( void )
{
	if (gFoundation == nullptr)
		return;

	gPhysicsScene->release();

	gDispatcher->release();
	PxProfileZoneManager* profileZoneManager = gPhysics->getProfileZoneManager();

	gPhysics->release();	
	profileZoneManager->release();
	gCooking->release();
	gFoundation->release();

	gDispatcher = nullptr;
	gPhysics = nullptr;
	gCooking = nullptr;
	gFoundation = nullptr;
}

static DynamicEntity::Ptr 	addEntityBox( float mass, vec3 halfsize, vec3 position )
{
	DynamicEntity::Ptr entity(new DynamicEntity());

	entity->scale = halfsize * 2.f;
	entity->position = position;

	PxTransform pxtr(PxVec3(position.x, position.y, position.z), PxQuat(PxIdentity));
	entity->body = gPhysics->createRigidDynamic(pxtr);
	entity->body->createShape( PxBoxGeometry(halfsize.x, halfsize.y, halfsize.z), *gPhysicsMaterial );
	entity->body->userData = (void*)entity.get();

	PxRigidBodyExt::updateMassAndInertia(*entity->body, 10.f);
	entity->body->setMass(mass);

	gPhysicsScene->addActor(*entity->body);

	return entity;
}

void 	updateStates( void )
{
	PxU32 nbActors = gPhysicsScene->getNbActors(
			PxActorTypeSelectionFlag::eRIGID_DYNAMIC);// | PxActorTypeSelectionFlag::eRIGID_STATIC);

	if(nbActors)
	{
		std::vector<PxRigidActor*> actors(nbActors);
		gPhysicsScene->getActors(
				PxActorTypeSelectionFlag::eRIGID_DYNAMIC/* | PxActorTypeSelectionFlag::eRIGID_STATIC*/, 
				(PxActor**)&actors[0], nbActors);

		for (PxRigidActor* actor : actors)
		{
			PxTransform localTm = actor->getGlobalPose();
			DynamicEntity* entity = (DynamicEntity*)actor->userData;
			entity->position = toVec3(localTm.p);
			entity->rotation = toQuat(localTm.q);
		}
	}
}


StaticEntity::Ptr 		initGround( vec3 halfsize, vec3 position )
{
	StaticEntity::Ptr ground(new StaticEntity());
	StaticEntity& e = *ground;
	e.scale = halfsize * 2.f;
	e.position = position;

	PxTransform pxtr(PxVec3(position.x, position.y, position.z), PxQuat(PxIdentity));
	e.body = gPhysics->createRigidStatic(pxtr);
	e.body->createShape( PxBoxGeometry(halfsize.x, halfsize.y, halfsize.z), *gPhysicsMaterial );

	gPhysicsScene->addActor(*e.body);
	return ground;
}


//// My workaround functions ////

static std::vector<PxFilterData> 	getFilterData( DynamicEntity& entity )
{
	PxRigidDynamic* dyn = entity.body;

	std::vector<PxShape*> 	shapes(dyn->getNbShapes());
	dyn->getShapes(&shapes[0], shapes.size());

	std::vector<PxFilterData> 	filterData(shapes.size());

	for (int i = 0; i < shapes.size(); ++i)
		filterData[i] = shapes[i]->getSimulationFilterData();
	return filterData;
}


static void 	setFilterData( DynamicEntity& entity, const std::vector<PxFilterData>& filterData )
{
	PxRigidDynamic* dyn = entity.body;

	std::vector<PxShape*> 	shapes(dyn->getNbShapes());
	dyn->getShapes(&shapes[0], shapes.size());

	for (int i = 0; i < shapes.size(); ++i)
		shapes[i]->setSimulationFilterData(filterData[i]);
}

//// Function for creating joints ////

void 	addFixedJoint( DynamicEntity& entityA, vec3 posA, DynamicEntity& entityB, vec3 posB, bool useWorkaround=false )
{
	//gPhysicsScene->removeActor(*entityA.body);
	//gPhysicsScene->addActor(*entityA.body);

	//entityA.body->clearForce();
	//entityA.body->clearForce(PxForceMode::eIMPULSE);
	//entityA.body->clearTorque();
	//entityA.body->clearTorque(PxForceMode::eIMPULSE);
	//entityA.body->setLinearVelocity(PxVec3(0, 0, 0));
	//entityA.body->setAngularVelocity(PxVec3(0, 0, 0));
	//PxRigidBodyExt::updateMassAndInertia(*entityA.body, 10.f);


	PxTransform otherPXTr = entityB.body->getGlobalPose();
	PxTransform meAnchor( toPxVec3(posA), PxQuat(PxIdentity) );
	PxTransform otherAnchor( toPxVec3(posB), PxQuat(PxIdentity) );

	PxTransform newMeTr = meAnchor.getInverse() * otherPXTr * otherAnchor;
	entityA.body->setGlobalPose(newMeTr);

	PxFixedJoint* joint = PxFixedJointCreate(
			*gPhysics, entityB.body, otherAnchor, entityA.body, meAnchor);
				
	if (useWorkaround)
	{
		auto fd = getFilterData(entityA);
		joint->setConstraintFlag( PxConstraintFlag::eCOLLISION_ENABLED, false );
		setFilterData(entityA, fd);
	}
	else
		joint->setConstraintFlag( PxConstraintFlag::eCOLLISION_ENABLED, false );
}

int 	main ( void )
{
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
	{
		std::cerr << "failed to load SDL. (everything)";
		return 1;
	}

	Graphics graphics;

	if (graphics.init(1280, 720) == false)
		return 1;

	if (initPhysics() == false)
		return 0;

	StaticEntity::Ptr ground = initGround(vec3(90.f, 0.5f, 90.f), VEC3_ZERO);

	// 'C' is used to make 'B' stands above the ground so that no collision will
	// interfere between 'A' and the ground when A will be fixed to B.
	DynamicEntity::Ptr C = addEntityBox(1000.f, vec3(8.f, 0.25f, 1.5f), vec3(0.f, 2.0, 0.f));
	DynamicEntity::Ptr B = addEntityBox(1000.f, vec3(8.f, 0.25f, 1.5f), vec3(0.f, 4.f, 0.f));
	addFixedJoint(*C, vec3(0.f, 1.f, 0.f), *B, vec3(0.f, -1.f, 0.f));

	DynamicEntity::Ptr A = addEntityBox(50.f, vec3(0.5f, 0.5f, 0.5f), vec3(0.f, 5.f, 0.f));

	auto t0 = std::chrono::high_resolution_clock::now();
	bool createJoint = false;
	while (true)
	{
		SDL_Event 	ev;
		SDL_PollEvent( &ev );
		if (ev.type == SDL_QUIT || (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
			break;

		auto t1 = std::chrono::high_resolution_clock::now();
		if (!createJoint && std::chrono::duration<float>(t1-t0).count() > 3.f)
		{
			debugDisplayFilterData(*A);
			debugDisplayFilterData(*B);
			addFixedJoint(*A, vec3(0.f, 0.f, 0.f), *B, vec3(0.f, 0.f, 0.f),	/*WORKAROUND-->*/ false);
			debugDisplayFilterData(*A);
			debugDisplayFilterData(*B);

			createJoint = true;
		}

		gPhysicsScene->simulate(1.f/60.f);
		gPhysicsScene->fetchResults(true);

		updateStates();
		graphics.clear();

		// Ground
		graphics.drawBox(ground->getModelMatrix(), Color(0.2f, 0.2f, 1.f));
		graphics.drawBox(A->getModelMatrix(), Color(0.2f, 1.f, 0.2f));
		graphics.drawBox(B->getModelMatrix(), Color(1.f, 0.2f, 0.2f));
		graphics.drawBox(C->getModelMatrix(), Color(1.f, 0.2f, 0.2f));

		graphics.refresh();
		usleep(1000);
	}

	graphics.deinit();
	deinitPhysics();

	SDL_Quit();

	return 0;
}

