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

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

#include "../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"

#include <cmath>
#include <ctime>
#include <iostream>
#include <array>

#define USE_RIGID_BODY 1
#define RENDER_RAYS 1

struct OrganProperties
{
    OrganProperties(float acoustic_impedance)
        : acoustic_impedance(acoustic_impedance)
    {}

    static constexpr float void_acoustic_impedance = 1.0f;
    float acoustic_impedance;
};

struct SoftBodyFromObjExample : public CommonRigidBodyBase {
    SoftBodyFromObjExample(struct GUIHelperInterface* helper):CommonRigidBodyBase(helper), frame_start(clock()) {}
    virtual ~SoftBodyFromObjExample(){}
    virtual void initPhysics();
    virtual void renderScene();
    void castRays();
    virtual void stepSimulation(float deltaTime);
    void createEmptyDynamicsWorld()
    {
#if 0
        m_collisionConfiguration = new btDefaultCollisionConfiguration();

        m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

        m_broadphase = new btDbvtBroadphase();

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
        m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);


        m_dynamicsWorld->setGravity(btVector3(0,-10,0));
#else
        m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
        m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);


        btVector3 aabb_min(-10,-10,-10);
        btVector3 aabb_max(10,10,10);
        m_broadphase = new btAxisSweep3(aabb_min, aabb_max);
        //m_broadphase = new btDbvtBroadphase();

        m_solver = new btSequentialImpulseConstraintSolver;

        m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

        softBodyWorldInfo.m_broadphase = m_broadphase;
        softBodyWorldInfo.m_dispatcher = m_dispatcher;
        softBodyWorldInfo.m_gravity = m_dynamicsWorld->getGravity();
        softBodyWorldInfo.m_sparsesdf.Initialize();
#endif
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

private:
    void add_softbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling);
    void add_rigidbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling);

    clock_t frame_start;
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
    struct mesh
    {
        std::string filename;
        bool is_rigid;
        std::array<float,3> deltas;
    };

    std::string working_dir = "data/ultrasound/";

    std::array<mesh,11> meshes
    {{
        {"aorta.obj", true, {152.533512115f, 174.472991943f, 105.106495678f}},
        {"bones.obj", true, {188.265544891f, 202.440551758f, 105.599998474f}},
        {"liver.obj", true, {141.238292694f, 176.429901123f, 130.10585022f}},
        {"cava.obj", true,  {206.332504272f, 192.29649353f, 104.897496045f}},
        {"right_kidney.obj", true, {118.23374939f, 218.907501221f, 53.6022927761f}},
        {"left_kidney.obj", true,  {251.052993774f, 227.63949585f, 64.8468027115f}},
        {"right_suprarrenal.obj", true, {152.25050354f, 213.971496582f, 115.338005066f}},
        {"left_suprarrenal.obj", true,  {217.128997803f, 209.525497437f, 102.477149963f}},
        {"gallbladder.obj", true, {128.70715332f, 146.592498779f, 112.361503601f}},
        {"skin.obj", true,  {188.597551346f, 199.367202759f, 105.622316509f}},
        {"porta.obj", true, {182.364089966f, 177.214996338f, 93.0034988523f}}
    }};

    for (const auto & mesh : meshes)
    {
        const auto full_path = working_dir + mesh.filename;

        if (mesh.is_rigid)
        {
            add_rigidbody_from_obj(full_path.c_str(), mesh.deltas, 0.1f);
        }
        else
        {
            add_softbody_from_obj(full_path.c_str(), mesh.deltas, 0.1f);
        }
    }

    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void SoftBodyFromObjExample::renderScene()
{
    CommonRigidBodyBase::renderScene();

//#if USE_RIGID_BODY == 0
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
//#endif
}

void SoftBodyFromObjExample::castRays()
{
    ///step the simulation
    if (m_dynamicsWorld)
    {

        m_dynamicsWorld->updateAabbs();
        m_dynamicsWorld->computeOverlappingPairs();

        constexpr size_t ray_count = 500;
        constexpr unsigned int max_ray_hits = 5;
        const float ray_length = 100;
        const btVector3 initial_pos(-50,1.2,-3);
        const float ray_start_step = 0.02f;

        const btVector3 empty_vector(0,0,0);
        std::array<std::array<btVector3, max_ray_hits>, ray_count> collisions;
        for (auto & ray : collisions)
        {
            for (auto & spot : ray)
            {
                spot = empty_vector;
            }
        }

        for (size_t ray_i = 0; ray_i < ray_count; ray_i++)
        {
            float current_acoustic_impedance = OrganProperties::void_acoustic_impedance;

            btVector3 from = initial_pos + btVector3(0,0,ray_start_step * ray_i);
            btVector3 ray_direction(1, 0, 0);

            for (unsigned int hit_i = 0; hit_i < max_ray_hits; hit_i++)
            {
                btVector3 to = from * 1.02f + ray_direction * ray_length;

                btCollisionWorld::ClosestRayResultCallback closestResults(from,to);

                m_dynamicsWorld->rayTest(from,to,closestResults);

                if (closestResults.hasHit())
                {
#if RENDER_RAYS
                    m_dynamicsWorld->getDebugDrawer()->drawLine(from,closestResults.m_hitPointWorld,btVector4(1,0,1,1));
#endif

                    OrganProperties * organ_properties = static_cast<OrganProperties*>(closestResults.m_collisionObject->getUserPointer());
                    float acoustic_impedance = organ_properties ? organ_properties->acoustic_impedance : current_acoustic_impedance;

                    const btVector3 refraction_direction = snells_law(ray_direction, closestResults.m_hitNormalWorld, current_acoustic_impedance, acoustic_impedance);
                    ray_direction = refraction_direction;

                    current_acoustic_impedance = acoustic_impedance;
                    from = closestResults.m_hitPointWorld;

                    collisions[ray_i][hit_i] = closestResults.m_hitPointWorld;
                }
                else
                {
                    break;
                }
            }
        }

        // draw
#if RENDER_RAYS
        btVector3 red(1,0,0);
        btVector3 blue(0,0,1);

        for (size_t ray_i = 0; ray_i < ray_count; ray_i++)
        {
            for (size_t hit_i = 0; hit_i < max_ray_hits; hit_i++)
            {
                if (collisions[ray_i][hit_i] != empty_vector)
                {
                    m_dynamicsWorld->getDebugDrawer()->drawSphere(collisions[ray_i][hit_i],0.1,hit_i%2?red:blue);
                }
                else
                {
                    break;
                }
            }
        }
#endif
    }

    const float fps = 1.0f / (float( clock() - frame_start ) /  CLOCKS_PER_SEC);
    std::cout << fps << std::endl;
    frame_start = clock();
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

void SoftBodyFromObjExample::add_softbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling)
{
    std::vector<tinyobj::shape_t> shapes;
    std::string err = tinyobj::LoadObj(shapes, fileName, "");

    if (!err.empty())
    {
        std::cout << err << std::endl;
        return;
    }
    else
    {
        std::cout << "File " << fileName << "loaded. shapes.size(): " << shapes.size() << std::endl;
    }

    btAlignedObjectArray<btScalar> vertices;
    btAlignedObjectArray<int> indices;

    //loop through all the shapes and add vertices and indices
    int offset = 0;
    for(unsigned int i=0;i<shapes.size();++i) {
        const tinyobj::shape_t& shape = shapes[i];

        //add vertices
        vertices.reserve(shape.mesh.positions.size());
        for(unsigned int j=0;j<shape.mesh.positions.size();++j) {
            vertices.push_back(shape.mesh.positions[j]);
        }

        //add indices
        indices.reserve(shape.mesh.indices.size());
        for(unsigned int j=0;j<shape.mesh.indices.size();++j) {
            indices.push_back(offset + shape.mesh.indices[j]);
        }
        offset += shape.mesh.positions.size();
    }
    printf("[INFO] Obj loaded: Extracted %d vertices, %d indices from obj file [%s]\n", vertices.size(), indices.size(), fileName);

    btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(softBodyWorldInfo, &vertices[0], &(indices[0]), indices.size()/3);

    OrganProperties * properties = new OrganProperties(0.6f);
    psb->setUserPointer(properties);

    btSoftBody::Material* pm=psb->appendMaterial();
    pm->m_kLST =	0.75;
    pm->m_flags -= btSoftBody::fMaterial::DebugDraw;
    psb->scale(btVector3(scaling, scaling, scaling));
    psb->generateBendingConstraints(4,pm);
    psb->m_cfg.piterations = 2;
    psb->m_cfg.kDF = 0.75;
    psb->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
    psb->randomizeConstraints();

    btMatrix3x3	m;
    //btVector3 x(-25,10,-7);
    //btVector3 x(deltas[0], deltas[1], deltas[2]);
    btVector3 x(0,0,0);
    btVector3 a(0,0,0);
    m.setEulerZYX(a.x(),a.y(),a.z());
    psb->transform(btTransform(m,x));
    psb->setTotalMass(1);
    psb->getCollisionShape()->setMargin(0.1f);
    getSoftDynamicsWorld()->addSoftBody(psb);
}

void SoftBodyFromObjExample::add_rigidbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling)
{
    GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(fileName, "");
    printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh->m_numvertices, fileName);

    const GLInstanceVertex& v = glmesh->m_vertices->at(0);
    btTriangleIndexVertexArray* tiva = new btTriangleIndexVertexArray(glmesh->m_numIndices / 3, &glmesh->m_indices->at(0), 3* sizeof(int),
                                                                      glmesh->m_numvertices, (btScalar*)(&(v.xyzw[0])), sizeof(GLInstanceVertex));

    btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(tiva, true);

    m_collisionShapes.push_back(shape);

    float _scaling[4] = {scaling,scaling,scaling,1};

    btVector3 localScaling(_scaling[0],_scaling[1],_scaling[2]);
    shape->setLocalScaling(localScaling);

    btTransform startTransform;
    startTransform.setIdentity();

    btScalar	mass(0.f);
    bool isDynamic = (mass != 0.f);
    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    float color[4] = {1,1,1,1};
    float orn[4] = {0,0,0,1};

    std::array<float, 3> origin { -18, -22, -5 };
    float pos[4] = {deltas[0]*_scaling[0]*_scaling[0],deltas[1]*_scaling[1]*_scaling[1],deltas[2]*_scaling[2]*_scaling[2],0};
    btVector3 position(pos[0] + origin[0], pos[1] + origin[1], pos[2] + origin[2]);
    startTransform.setOrigin(position);
        btRigidBody* body = createRigidBody(mass,startTransform,shape);

//    bool useConvexHullForRendering = false;//((m_options & ObjUseConvexHullForRendering)!=0);


//    if (!useConvexHullForRendering)
//    {
//        int shapeId = m_guiHelper->registerGraphicsShape(&glmesh->m_vertices->at(0).xyzw[0],
//                                                                        glmesh->m_numvertices,
//                                                                        &glmesh->m_indices->at(0),
//                                                                        glmesh->m_numIndices,
//                                                                        B3_GL_TRIANGLES, -1);
//        shape->setUserIndex(shapeId);
//        int renderInstance = m_guiHelper->registerGraphicsInstance(shapeId,pos,orn,color,scaling);
//        body->setUserIndex(renderInstance);
//    }
}

CommonExampleInterface*    ET_SoftBodyFromObjCreateFunc(CommonExampleOptions& options)
{
    return new SoftBodyFromObjExample(options.m_guiHelper);
}
