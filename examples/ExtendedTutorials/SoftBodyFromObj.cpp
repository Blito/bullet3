#include "SoftBodyFromObj.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

#include "../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"

#include <cmath>

struct SoftBodyFromObjExample : public CommonRigidBodyBase {
    SoftBodyFromObjExample(struct GUIHelperInterface* helper):CommonRigidBodyBase(helper) {}
    virtual ~SoftBodyFromObjExample(){}
    virtual void initPhysics();
    virtual void renderScene();
    void castRays();
    virtual void stepSimulation(float deltaTime);
    void createEmptyDynamicsWorld()
    {
        m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

        m_broadphase = new btDbvtBroadphase();

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

        softBodyWorldInfo.m_broadphase = m_broadphase;
        softBodyWorldInfo.m_dispatcher = m_dispatcher;
        softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
        softBodyWorldInfo.m_sparsesdf.Initialize();
    }
    virtual btSoftRigidDynamicsWorld* getSoftDynamicsWorld()
    {
        ///just make it a btSoftRigidDynamicsWorld please
        ///or we will add type checking
        return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
    }
    void resetCamera()
    {
        float dist = 13.6f;
        float pitch = 105.6f;
        float yaw = 18.0f;
        float targetPos[3]={3.573,1.454,-0.738};
        m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
    }

    btVector3 snells_law(btVector3 ray_direction, btVector3 surface_normal, float refr_index_1, float refr_index_2);

    btSoftBodyWorldInfo softBodyWorldInfo;
};

void SoftBodyFromObjExample::initPhysics() {
    m_guiHelper->setUpAxis(1);
    createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (m_dynamicsWorld->getDebugDrawer())
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

    btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,-50,0));

    {
        btScalar mass(0.);
        createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
    }

    //load our obj mesh
    const char* fileName = "liver.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
    char relativeFileName[1024];
    if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024)) {
        char pathPrefix[1024];
        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
    }
    std::vector<tinyobj::shape_t> shapes;
    std::string err = tinyobj::LoadObj(shapes, relativeFileName, "");

    btAlignedObjectArray<btScalar> vertices;
    btAlignedObjectArray<int> indices;

    //loop through all the shapes and add vertices and indices
    int offset = 0;
    for(int i=0;i<shapes.size();++i) {
        const tinyobj::shape_t& shape = shapes[i];

        //add vertices
        for(int j=0;j<shape.mesh.positions.size();++j) {
            vertices.push_back(shape.mesh.positions[j]);
        }

        //add indices
        for(int j=0;j<shape.mesh.indices.size();++j) {
            indices.push_back(offset + shape.mesh.indices[j]);
        }
        offset += shape.mesh.positions.size();
    }
    printf("[INFO] Obj loaded: Extracted %d vertices, %d indices from obj file [%s]\n", vertices.size(), indices.size(), fileName);

    btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(softBodyWorldInfo, &vertices[0], &(indices[0]), indices.size()/3);

    btVector3 scaling(0.1, 0.1, 0.1);

    btSoftBody::Material* pm=psb->appendMaterial();
    pm->m_kLST =	0.75;
    pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
    psb->scale(scaling);
    psb->generateBendingConstraints(4,pm);
    psb->m_cfg.piterations = 2;
    psb->m_cfg.kDF = 0.75;
    psb->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
    psb->randomizeConstraints();

    btMatrix3x3	m;
    btVector3 x(-25,10,-7);
    btVector3 a(0,0,0);
    m.setEulerZYX(a.x(),a.y(),a.z());
    psb->transform(btTransform(m,x));
    psb->setTotalMass(1);
    psb->getCollisionShape()->setMargin(0.1f);
    getSoftDynamicsWorld()->addSoftBody(psb);

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SoftBodyFromObjExample::renderScene()
{
    CommonRigidBodyBase::renderScene();
    btSoftRigidDynamicsWorld* softWorld = getSoftDynamicsWorld();

    for ( int i=0;i<softWorld->getSoftBodyArray().size();i++)
    {
        btSoftBody* psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
        //if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
        {
            btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
            btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
        }
    }
}

void SoftBodyFromObjExample::castRays()
{

    static float up = 0.f;
    static float dir = 1.f;
    //add some simple animation
    //if (!m_idle)
    {
        up+=0.01*dir;

        if (btFabs(up)>2)
        {
            dir*=-1.f;
        }

        btTransform tr = m_dynamicsWorld->getCollisionObjectArray()[1]->getWorldTransform();
        static float angle = 0.f;
        angle+=0.01f;
        tr.setRotation(btQuaternion(btVector3(0,1,0),angle));
        m_dynamicsWorld->getCollisionObjectArray()[1]->setWorldTransform(tr);
    }

    ///step the simulation
    if (m_dynamicsWorld)
    {

        m_dynamicsWorld->updateAabbs();
        m_dynamicsWorld->computeOverlappingPairs();

        btVector3 red(1,0,0);
        btVector3 blue(0,0,1);

        const unsigned int max_ray_hits = 5;
        const float ray_length = 100;

        btVector3 from(-50,1.2,0);
        btVector3 ray_direction(1, 0, 0);

        for (unsigned int ray_i = 0; ray_i < max_ray_hits; ray_i++)
        {
            btVector3 to = from * 1.02f + ray_direction * ray_length;

            btCollisionWorld::ClosestRayResultCallback closestResults(from,to);

            m_dynamicsWorld->rayTest(from,to,closestResults);

            if (closestResults.hasHit())
            {

                m_dynamicsWorld->getDebugDrawer()->drawLine(from,closestResults.m_hitPointWorld,btVector4(1,0,1,1));

                const btVector3 refraction_direction = snells_law(ray_direction, closestResults.m_hitNormalWorld, 1, 0.6);
                ray_direction = refraction_direction;

                from = closestResults.m_hitPointWorld;
            }
            else
            {
                break;
            }
        }
    }

}

void SoftBodyFromObjExample::stepSimulation(float deltaTime)
{
    castRays();
    CommonRigidBodyBase::stepSimulation(deltaTime);
}

btVector3 SoftBodyFromObjExample::snells_law(btVector3 ray_direction, btVector3 surface_normal, float refr_index_1, float refr_index_2)
{
    // For more details, read https://en.wikipedia.org/wiki/Snell%27s_law#Vector_form
    float cos_sigma1 = ray_direction.dot(-surface_normal);
    if (cos_sigma1 < 0.0f)
    {
        cos_sigma1 = ray_direction.dot(surface_normal);
    }

    //btVector3 reflection_dir{ 1 + 2 * cos_sigma1 * surface_normal };

    const btVector3 & l = ray_direction;
    const btVector3 & n = surface_normal;
    const float c = cos_sigma1;
    const float r = refr_index_1 / refr_index_2;

    return btVector3( r * l + (r*c - std::sqrt(1 - r*r * (1 - c*c))) * n );
}

CommonExampleInterface*    ET_SoftBodyFromObjCreateFunc(CommonExampleOptions& options)
{
    return new SoftBodyFromObjExample(options.m_guiHelper);
}
