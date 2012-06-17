/**
 *  @file      RagDoll.cpp
 *  @brief     Implementation of the RagDoll class that encapsulates a simple rag-doll.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-02
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "RagDoll.h"

/////////////////////////////////////////////////////////////////////////////////////////

RagDoll::RagDoll( 
        btDynamicsWorld* ownerWorld,
        const btTransform& offset,
        btScalar mass,
        bool rigidJoints,
        btScalar scale
    )
    : Machine( ownerWorld )
{
    /////////////////////////////////////////////////////////////////////////////////////
    // Setup the body parts (geometries associated to rigid bodies)

    btScalar lengthScale = scale * btScalar( 1e-3 );  // Our dimensions in millimeters

    Head.CapsuleShape( this, mass, offset,
        btVector3( 0, 1600, 0 ),
        Const::Zero,
        /* Radius = */ 100, /* Length = */ 50, lengthScale
    );

    UpperTrunk.CapsuleShape( this, mass, offset,
        btVector3( 0, 1200, 0 ),
        Const::Zero,
        /* Radius = */ 150, /* Length = */ 280, lengthScale
    );

    Pelvis.CapsuleShape( this, mass, offset, 
        btVector3( 0, 1000, 0 ), 
        Const::Zero, 
        /* Radius = */ 150, /* Length = */ 200, lengthScale
    );

    LeftThigh.CapsuleShape( this, mass, offset,
        btVector3( -180, 650, 0 ),
        Const::Zero,
        /* Radius = */ 70, /* Length = */ 450, lengthScale
   );

    LeftShank.CapsuleShape( this, mass, offset,
        btVector3( -180, 200, 0 ),
        btVector3( 0, 0, 0),
        /* Radius = */ 50, /* Length = */ 370, lengthScale
    );

    RightThigh.CapsuleShape( this, mass, offset,
        btVector3( 180, 650, 0 ),
        Const::Zero,
        /* Radius = */ 70, /* Length = */ 450, lengthScale
    );

    RightShank.CapsuleShape( this, mass, offset,
        btVector3( 180, 200, 0 ),
        Const::Zero,
        /* Radius = */ 50, /* Length = */ 370, lengthScale
    );

    LeftUpperArm.CapsuleShape( this, mass, offset, 
        btVector3( -350, 1450, 0 ),
        btVector3( 0, 0, SIMD_HALF_PI ),
        /* Radius = */ 50, /* Length = */ 330, lengthScale
    );

    LeftForearm.CapsuleShape( this, mass, offset,
        btVector3( -700, 1450, 0 ),
        btVector3( 0, 0, SIMD_HALF_PI ),
        /* Radius = */ 40, /* Length = */ 250, lengthScale
    );

    RightUpperArm.CapsuleShape( this, mass, offset,
        btVector3( 350, 1450, 0 ),
        btVector3( 0, 0, -SIMD_HALF_PI ),
        /* Radius = */ 50, /* Length = */ 330, lengthScale
    );

    RightForearm.CapsuleShape( this, mass, offset,
        btVector3( 700, 1450, 0 ),
        btVector3( 0, 0, -SIMD_HALF_PI ),
        /* Radius = */ 40, /* Length = */ 250, lengthScale
    );

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup the constraints

    UpperTrunkHead.Create6DOF( this, 
        UpperTrunk, btVector3( 0,  300, 0 ),
        Head,       btVector3( 0, -140, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_PI * 0.3f, -SIMD_EPSILON, -SIMD_PI * 0.3f ),
        btVector3(  SIMD_PI * 0.5f,  SIMD_EPSILON,  SIMD_PI * 0.3f ),
        lengthScale
    );

    LeftShoulder.Create6DOF( this, 
        UpperTrunk,    btVector3( -200,  150, 0 ),
        LeftUpperArm,  btVector3(    0, -180, 0 ),
        btVector3( SIMD_HALF_PI, 0, -SIMD_HALF_PI ),
        rigidJoints,
        btVector3( -SIMD_PI * 0.8f, -SIMD_EPSILON, -SIMD_PI * 0.5f ),
        btVector3(  SIMD_PI * 0.8f,  SIMD_EPSILON,  SIMD_PI * 0.5f ),
        lengthScale
    );

    RightShoulder.Create6DOF( this, 
        UpperTrunk,    btVector3( 200,  150, 0 ),
        RightUpperArm, btVector3(   0, -180, 0),
        btVector3( 0, 0, SIMD_HALF_PI ),
        rigidJoints,
        btVector3( -SIMD_PI * 0.8f, -SIMD_EPSILON, -SIMD_PI * 0.5f ),
        btVector3(  SIMD_PI * 0.8f,  SIMD_EPSILON,  SIMD_PI * 0.5f ),
        lengthScale
    );

    LeftElbow.Create6DOF( this, 
        LeftUpperArm, btVector3( 0, 180, 0 ),
        LeftForearm, btVector3( 0, -140, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_EPSILON,   -SIMD_EPSILON, -SIMD_EPSILON ),
        btVector3(  SIMD_PI * 0.7f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    RightElbow.Create6DOF( this, 
        RightUpperArm, btVector3( 0,  180, 0 ),
        RightForearm,  btVector3( 0, -140, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_EPSILON,   -SIMD_EPSILON, -SIMD_EPSILON ),
        btVector3(  SIMD_PI * 0.7f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    PelvisUpperTrunk.Create6DOF( this, 
        Pelvis,      btVector3( 0,  150, 0 ),
        UpperTrunk,  btVector3( 0, -150, 0 ),
        btVector3( 0,0,0 ),
        rigidJoints,
        btVector3( -SIMD_PI * 0.3f, -SIMD_PI * 0.1f, -SIMD_PI * 0.2f ),
        btVector3(  SIMD_PI * 0.6f,  SIMD_PI * 0.1f,  SIMD_PI * 0.2f ),
        lengthScale
    );

    LeftHip.Create6DOF( this, 
        Pelvis,    btVector3( -180, -100, 0 ),
        LeftThigh, btVector3(    0,  225, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_HALF_PI * 0.5f, -SIMD_EPSILON, -SIMD_HALF_PI * 0.6f ),
        btVector3(  SIMD_HALF_PI * 0.8f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    RightHip.Create6DOF( this, 
        Pelvis,     btVector3( 180, -100, 0 ),
        RightThigh, btVector3(   0,  225, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_HALF_PI * 0.5f, -SIMD_EPSILON, -SIMD_HALF_PI * 0.6f ),
        btVector3(  SIMD_HALF_PI * 0.8f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    LeftKnee.Create6DOF( this, 
        LeftThigh, btVector3( 0, -225, 0 ),
        LeftShank, btVector3( 0,  185, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_EPSILON,   -SIMD_EPSILON, -SIMD_EPSILON ),
        btVector3(  SIMD_PI * 0.7f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    RightKnee.Create6DOF( this, 
        RightThigh, btVector3( 0, -225, 0 ),
        RightShank, btVector3( 0,  185, 0 ),
        Const::Zero,
        rigidJoints,
        btVector3( -SIMD_EPSILON,   -SIMD_EPSILON, -SIMD_EPSILON ),
        btVector3(  SIMD_PI * 0.7f,  SIMD_EPSILON,  SIMD_EPSILON ),
        lengthScale
    );

    EnableDeactivation ();
}
