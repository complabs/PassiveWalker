#pragma once
#ifndef _UTILITIES_H_INCLUDED
#define _UTILITIES_H_INCLUDED

/**
 *  @file      Utilities.h
 *  @brief     Definitions for various utility classes: Transform.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////
// Referenced BT classes

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

/////////////////////////////////////////////////////////////////////////////////////////

class btTransform;

/** Represents a rigid transform with only translation and rotation and no scaling/shear.
 */
class Transform : public btTransform
{
public:

    /** Constructs an identity transform.
     */
    Transform ()
    {
        setIdentity ();
    }

    /** Constructs an identity transform and sets the origin to the given offset.
     */
    Transform( const btVector3& offset )
    {
        setIdentity ();
        setOrigin( offset );
    }

    /** Constructs an identity transform and sets the origin to the given offset.
     */
    Transform( btScalar x, btScalar y, btScalar z )
    {
        setIdentity ();
        setOrigin( btVector3( x, y, z ) );
    }

    /** Set the matrix from Euler angles YPR around ZYX axes.
     */
    Transform& setEulerZYX( const btVector3& orientation )
    {
        if ( ! orientation.isZero () )
        {
            getBasis().setEulerZYX( 
                orientation.x (), orientation.y (), orientation.z ()
            );
        }
        return *this;
    }

    /** Set the matrix from Euler angles given in degrees around ZYX axes .
     */
    Transform& setEulerZYXdeg( btScalar x, btScalar y, btScalar z )
    {
        getBasis().setEulerZYX(
            SIMD_RADS_PER_DEG * x, SIMD_RADS_PER_DEG * y, SIMD_RADS_PER_DEG * z
        );
        return *this;
    }
};

#endif // _UTILITIES_H_INCLUDED
