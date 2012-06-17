/**
 *  @file      Machine.cpp
 *  @brief     Implementation of the Machine class.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-14
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Machine.h"
#include <cstdio>

/////////////////////////////////////////////////////////////////////////////////////////

// Displays the motion state of the underlying rigid body.
//
void Machine::Element::Dump( const char* title ) const
{
    if ( ! RigidBody ) {
        printf( "%s (without rigid body)\n", title );
        return;
    }

    const btVector3    X = RigidBody->getCenterOfMassPosition ();
    const btVector3    V = RigidBody->getLinearVelocity ();
    const btVector3    W = RigidBody->getAngularVelocity ();
    const btQuaternion Q = RigidBody->getOrientation ();
    const btScalar     R = Q.getAngle ();
    const btVector3    S = Q.getAxis ();

    #ifdef BT_USE_DOUBLE_PRECISION
        #define SCALAR_FMT "%+7.3lf"
        #define DEG_FMT    "%+6.1lf"
    #else
        #define SCALAR_FMT "%+7.3f"
        #define DEG_FMT    "%+6.1f"
    #endif
    #define VEC_FMT  SCALAR_FMT " " SCALAR_FMT " " SCALAR_FMT

    printf( "%s", title );
    printf( " |" VEC_FMT, X.x(), X.y(), X.z() );
    printf( " |" DEG_FMT, btDegrees( R ) );
    printf( " "  DEG_FMT, btDegrees( btAngle( Const::X, S ) ) );
    printf( " "  DEG_FMT, btDegrees( btAngle( Const::Y, S ) ) );
    printf( " "  DEG_FMT, btDegrees( btAngle( Const::Z, S ) ) );
    printf( " |" VEC_FMT, V.x(), V.y(), V.z() );
    printf( " |" VEC_FMT, W.x(), W.y(), W.z() );
    printf( "\n" );
            
    #undef VEC_FMT
    #undef DEG_FMT
    #undef SCALAR_FMT
}

