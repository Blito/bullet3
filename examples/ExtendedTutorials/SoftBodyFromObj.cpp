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
#include <unordered_map>
#include <vector>

#include "material.h"
#include "ray.h"

#include <omp.h>

#define USE_RIGID_BODY 1
#define RENDER_RAYS 1

struct SoftBodyFromObjExample : public CommonRigidBodyBase {
    SoftBodyFromObjExample(struct GUIHelperInterface* helper):CommonRigidBodyBase(helper), frame_start(clock()) {}
    virtual ~SoftBodyFromObjExample(){}
    virtual void initPhysics();
    virtual void renderScene();
    void castRays();
    virtual void stepSimulation(float deltaTime);
    void createEmptyDynamicsWorld()
    {
#if USE_RIGID_BODY
        m_collisionConfiguration = new btDefaultCollisionConfiguration();

        m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

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

    btSoftBodyWorldInfo softBodyWorldInfo;

private:
    struct mesh
    {
        std::string filename;
        bool is_rigid;
        std::array<float,3> deltas;
        material_id material;
    };

    struct organ_properties
    {
        organ_properties(material_id mat_id)
            : mat_id(mat_id)
        {}

        static constexpr float void_acoustic_impedance = 1.0f;
        material_id mat_id;
    };

    btSoftBody * add_softbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling);
    btRigidBody * add_rigidbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling);

    unsigned int max_ray_length(material_id mat, float intensity) const;

    float distance_in_mm(const btVector3 & v1, const btVector3 & v2) const;

    btVector3 enlarge(const btVector3 & versor, float mm) const;

    clock_t frame_start;

    const std::string working_dir { "data/ultrasound/" };
    const float frequency { 5.0f };
    const float intensity_epsilon { 1e-8 };
    const float initial_intensity { 1.0f };

    const std::array<float, 3> spacing {{ 1.0f, 1.0f, 1.0f }};

    std::unordered_map<material_id, material, EnumClassHash> materials
    {
        { material_id::GEL, {1.99f, 1e-8, 0.0f, 0.0f, 0.0f} },
        { material_id::AIR, {0.0004f, 1.64f, 0.78f, 0.56f, 0.1f} },
        { material_id::FAT, {1.38f, 0.63f, 0.5f, 0.5f, 0.0f} },
        { material_id::BONE, {7.8f, 5.0f, 0.78f, 0.56f, 0.1f} },
        { material_id::BLOOD, {1.61f, 0.18f, 0.001f, 0.0f, 0.01f} },
        { material_id::VESSEL, {1.99f, 1.09f, 0.2f, 0.1f, 0.2f} },
        { material_id::LIVER, {1.65f, 0.7f, 0.19f, 1.0f, 0.24f} },
        { material_id::KIDNEY, {1.62f, 1.0f, 0.4f, 0.6f, 0.3f} },
        { material_id::SUPRARRENAL, {1.62f, 1.0f, 0.4f, 0.6f, 0.3f} }, // todo
        { material_id::GALLBLADDER, {1.62f, 1.0f, 0.4f, 0.6f, 0.3f} }, // todo
        { material_id::SKIN, {1.99f, 0.6f, 0.5f, 0.2f, 0.5f} },
    };
};

void SoftBodyFromObjExample::initPhysics() {
    m_guiHelper->setUpAxis(1);
    createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (m_dynamicsWorld->getDebugDrawer())
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

    std::array<mesh,11> meshes
    {{
        {"aorta.obj", true, {152.533512115f, 174.472991943f, 105.106495678f}, material_id::BLOOD},
        {"bones.obj", true, {188.265544891f, 202.440551758f, 105.599998474f}, material_id::BONE},
        {"liver.obj", true, {141.238292694f, 176.429901123f, 130.10585022f}, material_id::LIVER},
        {"cava.obj", true,  {206.332504272f, 192.29649353f, 104.897496045f}, material_id::BLOOD},
        {"right_kidney.obj", true, {118.23374939f, 218.907501221f, 53.6022927761f}, material_id::KIDNEY},
        {"left_kidney.obj", true,  {251.052993774f, 227.63949585f, 64.8468027115f}, material_id::KIDNEY},
        {"right_suprarrenal.obj", true, {152.25050354f, 213.971496582f, 115.338005066f}, material_id::SUPRARRENAL},
        {"left_suprarrenal.obj", true,  {217.128997803f, 209.525497437f, 102.477149963f}, material_id::SUPRARRENAL},
        {"gallbladder.obj", true, {128.70715332f, 146.592498779f, 112.361503601f}, material_id::GALLBLADDER},
        {"skin.obj", true,  {188.597551346f, 199.367202759f, 105.622316509f}, material_id::BONE},
        {"porta.obj", true, {182.364089966f, 177.214996338f, 93.0034988523f}, material_id::BLOOD}
    }};

    for (const auto & mesh : meshes)
    {
        const auto full_path = working_dir + mesh.filename;

        auto object = mesh.is_rigid ?
            static_cast<btCollisionObject*>(add_rigidbody_from_obj(full_path.c_str(), mesh.deltas, 0.1f)):
            static_cast<btCollisionObject*>(add_softbody_from_obj(full_path.c_str(), mesh.deltas, 0.1f));

        auto properties = new organ_properties(mesh.material);
        object->setUserPointer(properties);
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
    using namespace ray_physics;

    unsigned int tests = 0;
    unsigned int total_collisions = 0;
    constexpr bool render_rays = false;
    constexpr bool render_hits = false;
    ///step the simulation
    if (m_dynamicsWorld)
    {
        constexpr size_t ray_count { 256 };
        btVector3 initial_pos(-14,1.2,-3);
        const float ray_start_step { 0.02f };

        std::array<std::vector<collision>, ray_count> collisions;
        for (auto & collision_vector : collisions)
        {
            collision_vector.reserve(std::pow(2, ray::max_depth));
        }

        #pragma omp parallel for
        for (size_t ray_i = 0; ray_i < ray_count; ray_i++)
        {
            std::vector<ray> ray_stack;
            ray_stack.reserve(ray::max_depth-1);

            // Add first ray
            {
                ray first_ray
                {
                    initial_pos + btVector3(0,0,ray_start_step * ray_i), // from
                    {1, 0, 0},                                           // initial direction
                    0,                                                   // depth
                    materials[material_id::GEL],
                    initial_intensity,
                    frequency,
                    0                                                    // previous ray
                };
                ray_stack.push_back(first_ray);
            }

            while (ray_stack.size() > 0)
            {
                // Pop a ray from the stack and check if it collides

                auto ray_ = ray_stack.at(ray_stack.size()-1);

                float r_length = ray_physics::max_ray_length(ray_);
                auto to = ray_.from * 1.02f + enlarge(ray_.direction, r_length);

                btCollisionWorld::ClosestRayResultCallback closestResults(ray_.from,to);

                m_dynamicsWorld->rayTest(ray_.from,to,closestResults);
                tests++;

                ray_stack.pop_back();

                if (closestResults.hasHit())
                {
                    organ_properties * organ = static_cast<organ_properties*>(closestResults.m_collisionObject->getUserPointer());
                    const auto & organ_material = organ ? materials[organ->mat_id] : ray_.media;

                    if (render_rays)
                    {
                        m_dynamicsWorld->getDebugDrawer()->drawLine(ray_.from,closestResults.m_hitPointWorld,btVector4(1,0,1,1));
                    }

                    // Substract ray intensity according to distance traveled
                    {
                        ray_physics::travel(ray_, distance_in_mm(ray_.from, closestResults.m_hitPointWorld));
                    }

                    if (ray_.depth < ray::max_depth)
                    {
                        // Calculate refraction and reflection directions and intensities

                        auto result = ray_physics::hit_boundary(ray_, closestResults.m_hitPointWorld, closestResults.m_hitNormalWorld, organ_material);

                        // Register collision
                        collisions[ray_i].push_back({closestResults.m_hitPointWorld, ray_.parent_collision});

                        // Spawn reflection and refraction rays
                        if (result.refraction.intensity > ray::intensity_epsilon)
                        {
                            result.refraction.parent_collision = collisions[ray_i].size()-1;
                            ray_stack.push_back(result.refraction);
                        }

                        if (result.reflection.intensity > ray::intensity_epsilon)
                        {
                            result.reflection.parent_collision = collisions[ray_i].size()-1;
                            ray_stack.push_back(result.reflection);
                        }
                    }
                }
                else
                {
                    // Ray did not reach another media, add a data point at its end.
                    collisions[ray_i].push_back({to, ray_.parent_collision});
                }
            }
        }

        // draw
        if (render_hits)
        {
            btVector3 red(1,0,0);
            btVector3 blue(0,0,1);

            for (auto & collision_vector : collisions)
            {
                for (auto & collision : collision_vector)
                {
                    m_dynamicsWorld->getDebugDrawer()->drawSphere(collision.position,0.1,red);
                }
            }
        }

        for (auto & collision_vector : collisions)
        {
            total_collisions += collision_vector.size();
        }
    }

    const float fps = 1.0f / (float( clock() - frame_start ) /  CLOCKS_PER_SEC);
    std::cout << fps << " " << tests << " " << total_collisions << std::endl;
    frame_start = clock();
}

void SoftBodyFromObjExample::stepSimulation(float deltaTime)
{
    castRays();
    CommonRigidBodyBase::stepSimulation(deltaTime);
}

btSoftBody * SoftBodyFromObjExample::add_softbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling)
{
    std::vector<tinyobj::shape_t> shapes;
    std::string err = tinyobj::LoadObj(shapes, fileName, "");

    if (!err.empty())
    {
        std::cout << err << std::endl;
        return nullptr;
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

    return psb;
}

btRigidBody * SoftBodyFromObjExample::add_rigidbody_from_obj(const char * fileName, std::array<float, 3> deltas, float scaling)
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

    std::array<float, 3> origin { -18, -22, -5 };
    float pos[4] = {deltas[0]*_scaling[0]*_scaling[0],deltas[1]*_scaling[1]*_scaling[1],deltas[2]*_scaling[2]*_scaling[2],0};
    btVector3 position(pos[0] + origin[0], pos[1] + origin[1], pos[2] + origin[2]);
    startTransform.setOrigin(position);

    return createRigidBody(mass,startTransform,shape);

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

float SoftBodyFromObjExample::distance_in_mm(const btVector3 & v1, const btVector3 & v2) const
{
    using namespace std;

    auto x_dist = abs(v1.getX() - v2.getX()) * spacing[0];
    auto y_dist = abs(v1.getY() - v2.getY()) * spacing[1];
    auto z_dist = abs(v1.getZ() - v2.getZ()) * spacing[2];

    return sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(z_dist,2));
}

btVector3 SoftBodyFromObjExample::enlarge(const btVector3 & versor, float mm) const
{
    assert(versor.length2() < 1.1f);

    return mm/100.0f * btVector3 ( spacing[0] * versor.getX(),
                                   spacing[1] * versor.getY(),
                                   spacing[2] * versor.getZ() );
}

CommonExampleInterface*    ET_SoftBodyFromObjCreateFunc(CommonExampleOptions& options)
{
    return new SoftBodyFromObjExample(options.m_guiHelper);
}
