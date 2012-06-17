/**
 *  @file      PassiveWalker.cpp
 *  @brief     Implementation of the PassiveWalker class that encapsulates
 *             a simple passive walker with knees.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "PassiveWalker.h"
#include "Constants.h"

#include <cstdlib> // rand, srand
#include <ctime>   // time
#include <cstdio>  // printf

/////////////////////////////////////////////////////////////////////////////////////////

// Creates a set of machine elements and joints for the walker according
// to the given construction info found in PassiveWalker::Parameters and
// with the initial conditions found in PassiveWalker::Inits.
//
void PassiveWalker::Create ()
{
    /////////////////////////////////////////////////////////////////////////////////////
    // Cache some offten used parameters

    btScalar legRadius        = Parameters.LegRadius;
    btScalar innerLegOffset   = Parameters.InnerLegOffset;
    btScalar outerLegOffset   = Parameters.OuterLegOffset;
    btScalar thighLength      = Parameters.ThighLength;
    btScalar shankLength      = Parameters.ShankLength;

    btScalar footRadius       = Parameters.FootRadius;
    btScalar footHeight       = Parameters.FootRadius;
    btScalar footWidth        = Parameters.FootWidth;
    btScalar footLength       = Parameters.FootLength;
    btScalar footOffsetZ      = Parameters.FootOffsetZ;
    btScalar footOffsetY      = Parameters.FootOffsetY;
    btScalar footOffsetX      = Parameters.FootOffsetX;

    btScalar kneeForwardAngle = btRadians( Parameters.KneeForwardAngle );
    btScalar innerThighAngle  = btRadians( Parameters.InnerThighAngle  );
    btScalar innerShankAngle  = btRadians( Parameters.InnerShankAngle  );
    btScalar outerThighAngle  = btRadians( Parameters.OuterThighAngle  );
    btScalar outerShankAngle  = btRadians( Parameters.OuterShankAngle  );

    /////////////////////////////////////////////////////////////////////////////////////
    // Calculate xyz offsets for the elements according to the given knee and hip angles

    btScalar hipAxisY = thighLength + shankLength + footOffsetY + footHeight;
    btScalar hipAxisZ = 0;

    btScalar footToShank = shankLength/2 + footOffsetY;

    /////////////////////////////////////////////////////////////////////////////////////

    btScalar innerThighY  =  hipAxisY 
                          -  thighLength/2 * btCos( innerThighAngle );

    btScalar innerThighZ  =  hipAxisZ
                          +  thighLength/2 * btSin( innerThighAngle );

    btScalar innerShankY  =  hipAxisY 
                          -  thighLength   * btCos( innerThighAngle ) 
                          -  shankLength/2 * btCos( innerShankAngle );

    btScalar innerShankZ  =  hipAxisZ
                          +  thighLength   * btSin( innerThighAngle ) 
                          +  shankLength/2 * btSin( innerShankAngle );

    btScalar innerFootY   =  hipAxisY 
                          -  thighLength   * btCos( innerThighAngle ) 
                          -  shankLength   * btCos( innerShankAngle ) 
                          -  footOffsetY   * btCos( innerShankAngle )
                          +  footOffsetZ   * btSin( innerShankAngle );

    btScalar innerFootZ   =  hipAxisZ
                          +  thighLength   * btSin( innerThighAngle ) 
                          +  shankLength   * btSin( innerShankAngle ) 
                          +  footOffsetY   * btSin( innerShankAngle )
                          +  footOffsetZ   * btCos( innerShankAngle );

    btScalar outerThighY  =  hipAxisY 
                          -  thighLength/2 * btCos( outerThighAngle );

    btScalar outerThighZ  =  hipAxisZ
                          +  thighLength/2 * btSin( outerThighAngle );

    btScalar outerShankY  =  hipAxisY 
                          -  thighLength   * btCos( outerThighAngle ) 
                          -  shankLength/2 * btCos( outerShankAngle );

    btScalar outerShankZ  =  hipAxisZ 
                          +  thighLength   * btSin( outerThighAngle ) 
                          +  shankLength/2 * btSin( outerShankAngle );

    btScalar outerFootY   =  hipAxisY 
                          -  thighLength   * btCos( outerThighAngle ) 
                          -  shankLength   * btCos( outerShankAngle ) 
                          -  footOffsetY   * btCos( outerShankAngle )
                          +  footOffsetZ   * btSin( outerShankAngle );

    btScalar outerFootZ   =  hipAxisZ
                          +  thighLength   * btSin( outerThighAngle ) 
                          +  shankLength   * btSin( outerShankAngle ) 
                          +  footOffsetY   * btSin( outerShankAngle )
                          +  footOffsetZ   * btCos( outerShankAngle );

    /////////////////////////////////////////////////////////////////////////////////////
    // Determine the gap bellow the feet and calculate the initial offset

    btVector3 gapBellowFeet;
    gapBellowFeet.setZero ();
    if ( outerFootY > innerFootY && innerFootY > footHeight ) {
        // inner foot is both the lowest and above ground
        gapBellowFeet.setY( innerFootY - footHeight );
    }
    else if ( innerFootY > outerFootY && outerFootY > footHeight ) {
        // outer foot is both the lowest and above ground
        gapBellowFeet.setY( outerFootY - footHeight );
    }

    if ( gapBellowFeet.y () != 0 ) {
        printf( "Walker's height: %lg\n", double( hipAxisY + footHeight ) );
        printf( "Gap bellow feet: %lg\n", double( gapBellowFeet.y () ) );
    }

    Transform offset( Parameters.Position - gapBellowFeet );
    offset.setEulerZYX( Parameters.EulerZYX );

    if ( Parameters.DisturbFixup ) {
        hipAxisY += 8;
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup machine elements (geometries and associated rigid bodies)

    HipAxis.CylinderShape( this, Parameters.HipMass, offset, 
        /* Position    */ btVector3( 0, hipAxisY, hipAxisZ ),
        /* Orientation */ btVector3( 0, 0, SIMD_HALF_PI ),
        /* Half-Extent */ btVector3( legRadius, outerLegOffset + legRadius, legRadius ), 
        Parameters.LengthUnits
    );

    Inner.Left.Thigh.BoxShape( this, Parameters.ThighMass, offset, 
        /* Position    */ btVector3( +innerLegOffset, innerThighY, innerThighZ ),
        /* Orientation */ btVector3( -innerThighAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, thighLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Inner.Right.Thigh.BoxShape( this, Parameters.ThighMass, offset, 
        /* Position    */ btVector3( -innerLegOffset, innerThighY, innerThighZ ),
        /* Orientation */ btVector3( -innerThighAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, thighLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Outer.Left.Thigh.BoxShape( this, Parameters.ThighMass, offset, 
        /* Position    */ btVector3( +outerLegOffset, outerThighY, outerThighZ ),
        /* Orientation */ btVector3( -outerThighAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, thighLength/2, legRadius ),
        Parameters.LengthUnits
    );

    Outer.Right.Thigh.BoxShape( this, Parameters.ThighMass, offset, 
        /* Position    */ btVector3( -outerLegOffset, outerThighY, outerThighZ ),
        /* Orientation */ btVector3( -outerThighAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, thighLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Inner.Left.Shank.BoxShape( this, Parameters.ShankMass, offset, 
        /* Position    */ btVector3( +innerLegOffset, innerShankY, innerShankZ ),
        /* Orientation */ btVector3( -innerShankAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, shankLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Inner.Right.Shank.BoxShape( this, Parameters.ShankMass, offset, 
        /* Position    */ btVector3( -innerLegOffset, innerShankY, innerShankZ ),
        /* Orientation */ btVector3( -innerShankAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, shankLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Outer.Left.Shank.BoxShape( this, Parameters.ShankMass, offset, 
        /* Position    */ btVector3( +outerLegOffset, outerShankY, outerShankZ ),
        /* Orientation */ btVector3( -outerShankAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, shankLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    Outer.Right.Shank.BoxShape( this, Parameters.ShankMass, offset, 
        /* Position    */ btVector3( -outerLegOffset, outerShankY, outerShankZ ),
        /* Orientation */ btVector3( -outerShankAngle, 0, 0 ), 
        /* Half-Extent */ btVector3( legRadius, shankLength/2, legRadius ), 
        Parameters.LengthUnits
    );

    if ( footLength == 0 ) // Cylindrical feet
    {
        Inner.Left.Foot.CylinderShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( +( innerLegOffset + footOffsetX ), innerFootY, innerFootZ ),
            /* Orientation */ btVector3( 0, innerShankAngle, SIMD_HALF_PI ),
            /* Half-Extent */ btVector3( footRadius, footWidth, footRadius ), 
            Parameters.LengthUnits
        );

        Inner.Right.Foot.CylinderShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( -( innerLegOffset + footOffsetX ), innerFootY, innerFootZ ),
            /* Orientation */ btVector3( 0, innerShankAngle, SIMD_HALF_PI ),
            /* Half-Extent */ btVector3( footRadius, footWidth, footRadius ), 
            Parameters.LengthUnits
        );

        Outer.Left.Foot.CylinderShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( +( outerLegOffset + footOffsetX ), outerFootY, outerFootZ ),
            /* Orientation */ btVector3( 0, outerShankAngle, SIMD_HALF_PI ),
            /* Half-Extent */ btVector3( footRadius, footWidth, footRadius ), 
            Parameters.LengthUnits
        );

        Outer.Right.Foot.CylinderShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( -( outerLegOffset + footOffsetX ), outerFootY, outerFootZ ),
            /* Orientation */ btVector3( 0, outerShankAngle, SIMD_HALF_PI ),
            /* Half-Extent */ btVector3( footRadius, footWidth, footRadius ), 
            Parameters.LengthUnits
        );
    }
    else // Cuboid feet
    {
        Inner.Left.Foot.BoxShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( +( innerLegOffset + footOffsetX ), innerFootY, innerFootZ ),
            /* Orientation */ btVector3( 0, 0, 0 ),
            /* Half-Extent */ btVector3( footWidth, footHeight, footLength ), 
            Parameters.LengthUnits
        );

        Inner.Right.Foot.BoxShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( -( innerLegOffset + footOffsetX ), innerFootY, innerFootZ ),
            /* Orientation */ btVector3( 0, 0, 0 ),
            /* Half-Extent */ btVector3( footWidth, footHeight, footLength ), 
            Parameters.LengthUnits
        );

        Outer.Left.Foot.BoxShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( +( outerLegOffset + footOffsetX ), outerFootY, outerFootZ ),
            /* Orientation */ btVector3( 0, 0, 0 ),
            /* Half-Extent */ btVector3( footWidth, footHeight, footLength ), 
            Parameters.LengthUnits
        );

        Outer.Right.Foot.BoxShape( this, Parameters.FootMass, offset, 
            /* Position    */ btVector3( -( outerLegOffset + footOffsetX ), outerFootY, outerFootZ ),
            /* Orientation */ btVector3( 0, 0, 0 ),
            /* Half-Extent */ btVector3( footWidth, footHeight, footLength ), 
            Parameters.LengthUnits
        );
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup constraints between the machine elements (the joints)

    if ( ! Parameters.LockOuterLegsToHipAxis ) {
        HipAxis->setAngularFactor( Const::Y );
    }

    Inner.ThighLock.CreateFixed( this, 
        /* A, pos */ Inner.Left.Thigh,  btVector3( -innerLegOffset, 0, 0 ),
        /* B, pos */ Inner.Right.Thigh, btVector3( +innerLegOffset, 0, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.LengthUnits
    );

    Outer.ThighLock.CreateFixed( this, 
        /* A, pos */ Outer.Left.Thigh,  btVector3( -outerLegOffset, 0, 0 ),
        /* B, pos */ Outer.Right.Thigh, btVector3( +outerLegOffset, 0, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.LengthUnits
    );

    Inner.ShankLock.CreateFixed( this, 
        /* A, pos */ Inner.Left.Shank,  btVector3( -innerLegOffset, 0, 0 ),
        /* B, pos */ Inner.Right.Shank, btVector3( +innerLegOffset, 0, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.LengthUnits
    );

    Outer.ShankLock.CreateFixed( this, 
        /* A, pos */ Outer.Left.Shank,  btVector3( -outerLegOffset, 0, 0 ),
        /* B, pos */ Outer.Right.Shank, btVector3( +outerLegOffset, 0, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.LengthUnits
    );

    Inner.Left.Hip.Create6DOF( this, 
        /* A, pos */ Inner.Left.Thigh, btVector3( 0, thighLength/2, 0 ),
        /* B, pos */ HipAxis,          btVector3( 0, -innerLegOffset, 0 ),
        /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
        /* rigid */ false,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3(  SIMD_HALF_PI, 0, 0 ),
        Parameters.LengthUnits
    );

    Inner.Right.Hip.Create6DOF( this, 
        /* A, pos */ Inner.Right.Thigh, btVector3( 0, thighLength/2, 0 ),
        /* B, pos */ HipAxis,           btVector3( 0, +innerLegOffset, 0 ),
        /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
        /* rigid */ false,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3(  SIMD_HALF_PI, 0, 0 ),
        Parameters.LengthUnits
    );

    Outer.Left.Hip.Create6DOF( this, 
        /* A, pos */ Outer.Left.Thigh, btVector3( 0, thighLength/2, 0 ),
        /* B, pos */ HipAxis,          btVector3( 0, -outerLegOffset, 0 ),
        /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
        /* rigid */ Parameters.LockOuterLegsToHipAxis,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3(  SIMD_HALF_PI, 0, 0 ),
        Parameters.LengthUnits
    );

    Outer.Right.Hip.Create6DOF( this, 
        /* A, pos */ Outer.Right.Thigh, btVector3( 0, thighLength/2, 0 ),
        /* B, pos */ HipAxis,           btVector3( 0, +outerLegOffset, 0 ),
        /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
        /* rigid */ Parameters.LockOuterLegsToHipAxis,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3(  SIMD_HALF_PI, 0, 0 ),
        Parameters.LengthUnits
    );

    Inner.Left.Knee.Create6DOF( this, 
        /* A, pos */ Inner.Left.Thigh, btVector3( 0, -thighLength/2, 0 ),
        /* B, pos */ Inner.Left.Shank, btVector3( 0, shankLength/2, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.RigidKnees,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ), 
        /* upper angle limit */ btVector3( kneeForwardAngle, 0, 0 ), 
        Parameters.LengthUnits
    );

    Inner.Right.Knee.Create6DOF( this, 
        /* A, pos */ Inner.Right.Thigh, btVector3( 0, -thighLength/2, 0 ),
        /* B, pos */ Inner.Right.Shank, btVector3( 0, shankLength/2, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.RigidKnees,
        /* angle limits */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        btVector3( kneeForwardAngle, 0, 0 ),
        Parameters.LengthUnits
    );

    Outer.Left.Knee.Create6DOF( this, 
        /* A, pos */ Outer.Left.Thigh, btVector3( 0, -thighLength/2, 0 ),
        /* B, pos */ Outer.Left.Shank, btVector3( 0, shankLength/2, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.RigidKnees,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3( 
            Parameters.DisturbFixup ? btScalar( 0.1 ) : kneeForwardAngle, 0, 0 ),
        Parameters.LengthUnits
    );

    Outer.Right.Knee.Create6DOF( this, 
        /* A, pos */ Outer.Right.Thigh, btVector3( 0, -thighLength/2, 0 ),
        /* B, pos */ Outer.Right.Shank, btVector3( 0, shankLength/2, 0 ),
        /* B, rot */ Const::Zero,
        Parameters.RigidKnees,
        /* lower angle limit */ btVector3( -SIMD_HALF_PI, 0, 0 ),
        /* upper angle limit */ btVector3( kneeForwardAngle, 0, 0 ),
        Parameters.LengthUnits
    );

    if ( footLength == 0 ) // Cylindircal feet
    {
        Inner.Left.Ankle.CreateFixed( this, 
            /* A, pos */ Inner.Left.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Inner.Left.Foot,  btVector3( 0, +footOffsetX, 0 ),
            /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
            Parameters.LengthUnits
        );

        Inner.Right.Ankle.CreateFixed( this, 
            /* A, pos */ Inner.Right.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Inner.Right.Foot,  btVector3( 0, -footOffsetX, 0 ),
            /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
            Parameters.LengthUnits
        );

        Outer.Left.Ankle.CreateFixed( this, 
            /* A, pos */ Outer.Left.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Outer.Left.Foot,  btVector3( 0, +footOffsetX, 0 ),
            /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
            Parameters.LengthUnits
        );

        Outer.Right.Ankle.CreateFixed( this, 
            /* A, pos */ Outer.Right.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Outer.Right.Foot,  btVector3( 0, -footOffsetX, 0 ),
            /* B, rot */ btVector3( 0, 0, -SIMD_HALF_PI ),
            Parameters.LengthUnits
        );
    }
    else // Cuboid feet
    {
        Inner.Left.Ankle.Create6DOF( this, 
            /* A, pos */ Inner.Left.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Inner.Left.Foot,  btVector3( 0, +footOffsetX, 0 ),
            /* B, rot */ btVector3( 0, 0, 0 ), 
            /* rigid */ false,
            /* lower angle limit */ btVector3( -innerShankAngle, 0, 0 ),
            /* upper angle limit */ btVector3( -innerShankAngle, 0, 0 ),
            Parameters.LengthUnits
        );

        Inner.Right.Ankle.Create6DOF( this, 
            /* A, pos */ Inner.Right.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Inner.Right.Foot,  btVector3( 0, -footOffsetX, 0 ),
            /* B, rot */ Const::Zero,
            /* rigid */ false,
            /* lower angle limit */ btVector3( -innerShankAngle, 0, 0 ),
            /* upper angle limit */ btVector3( -innerShankAngle, 0, 0 ),
            Parameters.LengthUnits
        );

        Outer.Left.Ankle.Create6DOF( this, 
            /* A, pos */ Outer.Left.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Outer.Left.Foot,  btVector3( 0, +footOffsetX, 0 ),
            /* B, rot */ Const::Zero,
            /* rigid */ false,
            /* lower angle limit */ btVector3( -outerShankAngle, 0, 0 ),
            /* upper angle limit */ btVector3( -outerShankAngle, 0, 0 ),
            Parameters.LengthUnits
        );

        Outer.Right.Ankle.Create6DOF( this, 
            /* A, pos */ Outer.Right.Shank, btVector3( 0, -footToShank, footOffsetZ ),
            /* B, pos */ Outer.Right.Foot,  btVector3( 0, -footOffsetX, 0 ),
            /* B, rot */ Const::Zero,
            /* rigid */ false,
            /* lower angle limit */ btVector3( -outerShankAngle, 0, 0 ),
            /* upper angle limit */ btVector3( -outerShankAngle, 0, 0 ),
            Parameters.LengthUnits
        );
    }

    /////////////////////////////////////////////////////////////////////////////////////
    // Configure the friction and restitution of the hip and feet

    HipAxis->setFriction( 0 ); // No friction at hip

    Inner .Left  .Foot -> setFriction( Parameters.FootFriction );
    Inner .Right .Foot -> setFriction( Parameters.FootFriction );
    Outer .Left  .Foot -> setFriction( Parameters.FootFriction );
    Outer .Right .Foot -> setFriction( Parameters.FootFriction );

    Inner .Left  .Foot -> setRestitution( Parameters.FootRestitution );
    Inner .Right .Foot -> setRestitution( Parameters.FootRestitution );
    Outer .Left  .Foot -> setRestitution( Parameters.FootRestitution );
    Outer .Right .Foot -> setRestitution( Parameters.FootRestitution );

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup the initial linear velocities

    HipAxis             -> setLinearVelocity( Inits.V0.HipAxis    );
    Inner .Left  .Thigh -> setLinearVelocity( Inits.V0.InnerThigh );
    Inner .Right .Thigh -> setLinearVelocity( Inits.V0.InnerThigh );
    Inner .Left  .Shank -> setLinearVelocity( Inits.V0.InnerShank );
    Inner .Right .Shank -> setLinearVelocity( Inits.V0.InnerShank );
    Inner .Left  .Foot  -> setLinearVelocity( Inits.V0.InnerFoot  );
    Inner .Right .Foot  -> setLinearVelocity( Inits.V0.InnerFoot  );
    Outer .Left  .Thigh -> setLinearVelocity( Inits.V0.OuterThigh );
    Outer .Right .Thigh -> setLinearVelocity( Inits.V0.OuterThigh );
    Outer .Left  .Shank -> setLinearVelocity( Inits.V0.OuterShank );
    Outer .Right .Shank -> setLinearVelocity( Inits.V0.OuterShank );
    Outer .Left  .Foot  -> setLinearVelocity( Inits.V0.OuterFoot  );
    Outer .Right .Foot  -> setLinearVelocity( Inits.V0.OuterFoot  );

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup the initial angular velocities

    HipAxis             -> setAngularVelocity( Inits.W0.HipAxis    );
    Inner .Left  .Thigh -> setAngularVelocity( Inits.W0.InnerThigh );
    Inner .Right .Thigh -> setAngularVelocity( Inits.W0.InnerThigh );
    Inner .Left  .Shank -> setAngularVelocity( Inits.W0.InnerShank );
    Inner .Right .Shank -> setAngularVelocity( Inits.W0.InnerShank );
    Inner .Left  .Foot  -> setAngularVelocity( Inits.W0.InnerFoot  );
    Inner .Right .Foot  -> setAngularVelocity( Inits.W0.InnerFoot  );
    Outer .Left  .Thigh -> setAngularVelocity( Inits.W0.OuterThigh );
    Outer .Right .Thigh -> setAngularVelocity( Inits.W0.OuterThigh );
    Outer .Left  .Shank -> setAngularVelocity( Inits.W0.OuterShank );
    Outer .Right .Shank -> setAngularVelocity( Inits.W0.OuterShank );
    Outer .Left  .Foot  -> setAngularVelocity( Inits.W0.OuterFoot  );
    Outer .Right .Foot  -> setAngularVelocity( Inits.W0.OuterFoot  );

    /////////////////////////////////////////////////////////////////////////////////////

    DisableDeactivation ();

    if ( State == UNINITIALIZED ) {
        State = PASSIVE;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Connects the walker's contact sensors to the specified ground plane object.
//
void PassiveWalker::SetupGroundPlaneSensors( Machine::Element& GroundPlane )
{
    HipGroundContact.SetupPair( GroundPlane, HipAxis ); 
    Inner.Left .FootGroundContact.SetupPair( GroundPlane, Inner.Left .Foot );
    Inner.Right.FootGroundContact.SetupPair( GroundPlane, Inner.Right.Foot );
    Outer.Left .FootGroundContact.SetupPair( GroundPlane, Outer.Left .Foot );
    Outer.Right.FootGroundContact.SetupPair( GroundPlane, Outer.Right.Foot );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Returns the center of mass of the walker.
//
btVector3 PassiveWalker::GetCenterOfMass () const
{
    btScalar  totalMass  = 0;
    btVector3 centerMass = Const::Zero;

    for ( int i = 0; i < Elements.size (); ++i ) 
    {
        const Element& e = *Elements.at(i);

        if ( e->isStaticObject () ) {
            continue;
        }
        const btScalar mass = 1 / e->getInvMass ();
        const btVector3& pos = e->getCenterOfMassPosition ();
        centerMass += pos * mass;
        totalMass  += mass;
    }

    return centerMass / totalMass;
}

/////////////////////////////////////////////////////////////////////////////////////////

// Displays the walker's motion state variables on stdout.
//
void PassiveWalker::Dump( double localTime ) const
{
    printf( "--------------------------------------------------- " );
    printf( "t = %lg\n", localTime );
    HipAxis             .Dump( "Hip Axis" );
    Inner .Left  .Thigh .Dump( "IL Thigh" );
    Inner .Right .Thigh .Dump( "IR Thigh" );
    Inner .Left  .Shank .Dump( "IL Shank" );
    Inner .Right .Shank .Dump( "IR Shank" );
    Inner .Left  .Foot  .Dump( "IL Foot " );
    Inner .Right .Foot  .Dump( "IR Foot " );
    Outer .Left  .Thigh .Dump( "OL Thigh" );
    Outer .Right .Thigh .Dump( "OR Thigh" );
    Outer .Left  .Shank .Dump( "OL Shank" );
    Outer .Right .Shank .Dump( "OR Shank" );
    Outer .Left  .Foot  .Dump( "OL Foot " );
    Outer .Right .Foot  .Dump( "OR Foot " );
}

// Displays the contents of the initial conditions structure on stdout.
//
void PassiveWalker::InitialConditions::Dump () const
{
    printf( "---------------------------------------------------\n" );

    #define DUMP_VEC3_VERBOSE(v) \
        printf( "%-14s: %20.16lf %20.16lf %20.16lf\n", #v, \
            double( v.x() ), double( v.y() ), double( v.z() ) \
        )

    DUMP_VEC3_VERBOSE( V0.HipAxis    );
    DUMP_VEC3_VERBOSE( V0.InnerThigh );
    DUMP_VEC3_VERBOSE( V0.InnerShank );
    DUMP_VEC3_VERBOSE( V0.InnerFoot  );
    DUMP_VEC3_VERBOSE( V0.OuterThigh );
    DUMP_VEC3_VERBOSE( V0.OuterShank );
    DUMP_VEC3_VERBOSE( V0.OuterFoot  );

    DUMP_VEC3_VERBOSE( W0.HipAxis    );
    DUMP_VEC3_VERBOSE( W0.InnerThigh );
    DUMP_VEC3_VERBOSE( W0.InnerShank );
    DUMP_VEC3_VERBOSE( W0.InnerFoot  );
    DUMP_VEC3_VERBOSE( W0.OuterThigh );
    DUMP_VEC3_VERBOSE( W0.OuterShank );
    DUMP_VEC3_VERBOSE( W0.OuterFoot  );

    #undef DUMP_VEC3_VERBOSE
}

/////////////////////////////////////////////////////////////////////////////////////////

// Saves the initial conditions into a file 'config-[id].txt'.
//
void PassiveWalker::SaveInitialConditions( int id, double localTime ) const
{
    #ifdef _MSC_VER
        #pragma warning(disable:4996) // sprintf/fopen warning
    #endif

    char filename[ 256 ];
    sprintf( filename, "config-%d.txt", id );

    FILE* file = fopen( filename, "a" );
    if ( ! file ) {
        return;
    }

    fprintf( file, "\n------------------------------------------\n" );
    fprintf( file, "Test Suite ID: %d, Walk Duration: %lg s\n\n", id, localTime );

    fprintf( file, "%lg \n",      double( Inits.WalkDurationGoal      ) );
    fprintf( file, "%30.20le \n", double( Parameters.InclinationAngle ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.HipAxis   .getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.InnerThigh.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.InnerShank.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.InnerFoot .getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.OuterThigh.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.OuterShank.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.V0.OuterFoot .getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.HipAxis   .getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.InnerThigh.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.InnerShank.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.InnerFoot .getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.OuterThigh.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.OuterShank.getZ () ) );
    fprintf( file, "%30.20le \n", double( Inits.W0.OuterFoot .getZ () ) );

    fclose( file );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Loads initial conditions from file 'config-[id].txt'.
//
void PassiveWalker::LoadInitialConditions( int id )
{
    btScalar p[] = 
    {
        Inits.WalkDurationGoal,
        Parameters.InclinationAngle,
        Inits.V0.HipAxis   .getZ (),
        Inits.V0.InnerThigh.getZ (),
        Inits.V0.InnerShank.getZ (),
        Inits.V0.InnerFoot .getZ (),
        Inits.V0.OuterThigh.getZ (),
        Inits.V0.OuterShank.getZ (),
        Inits.V0.OuterFoot .getZ (),
        Inits.W0.HipAxis   .getZ (),
        Inits.W0.InnerThigh.getZ (),
        Inits.W0.InnerShank.getZ (),
        Inits.W0.InnerFoot .getZ (),
        Inits.W0.OuterThigh.getZ (),
        Inits.W0.OuterShank.getZ (),
        Inits.W0.OuterFoot .getZ (),
    };

    char filename[ 256 ];
    sprintf( filename, "config-%d.txt", id );

    FILE* file = fopen( filename, "r" );
    if ( file )
    {
        for ( int i = 0; i < sizeof(p)/sizeof(p[0]); ++i )
        {
            double parameter = 0;
            if ( 1 == fscanf( file, " %lg", &parameter ) ) {
                p[i] = btScalar( parameter );
            }
        }

        fclose( file );

        btScalar* pv = p;
        Inits.WalkDurationGoal = *pv++;
        Parameters.InclinationAngle = *pv++;
        Inits.V0.HipAxis    .setZ( *pv++ );
        Inits.V0.InnerThigh .setZ( *pv++ );
        Inits.V0.InnerShank .setZ( *pv++ );
        Inits.V0.InnerFoot  .setZ( *pv++ );
        Inits.V0.OuterThigh .setZ( *pv++ );
        Inits.V0.OuterShank .setZ( *pv++ );
        Inits.V0.OuterFoot  .setZ( *pv++ );
        Inits.W0.HipAxis    .setZ( *pv++ );
        Inits.W0.InnerThigh .setZ( *pv++ );
        Inits.W0.InnerShank .setZ( *pv++ );
        Inits.W0.InnerFoot  .setZ( *pv++ );
        Inits.W0.OuterThigh .setZ( *pv++ );
        Inits.W0.OuterShank .setZ( *pv++ );
        Inits.W0.OuterFoot  .setZ( *pv++ );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Randomizes the initial conditions uniformly 10% around the given center values.
//
void PassiveWalker::RandomizeInitialConditions ()
{
    // Setup seed for the rand function (once only)
    //
    static bool randomized = false;
    if ( ! randomized ) {
        randomized = true;
        srand( (unsigned)time( NULL ) );
    }

    #define MY_RND_GEN(x) ( 1 + btScalar(x) * ( btScalar(rand()) / RAND_MAX - 0.5f ) )

    const double range = 0.1;

    Parameters.InclinationAngle *= MY_RND_GEN( range/2 );

    Inits.V0.HipAxis    .setZ( Inits.V0.HipAxis    .getZ() * MY_RND_GEN( range ) );
    Inits.V0.InnerThigh .setZ( Inits.V0.InnerThigh .getZ() * MY_RND_GEN( range ) );
    Inits.V0.InnerShank .setZ( Inits.V0.InnerShank .getZ() * MY_RND_GEN( range ) );
    Inits.V0.InnerFoot  .setZ( Inits.V0.InnerFoot  .getZ() * MY_RND_GEN( range ) );
    Inits.V0.OuterThigh .setZ( Inits.V0.OuterThigh .getZ() * MY_RND_GEN( range ) );
    Inits.V0.OuterShank .setZ( Inits.V0.OuterShank .getZ() * MY_RND_GEN( range ) );

    Inits.W0.HipAxis    .setZ( Inits.W0.HipAxis    .getZ() * MY_RND_GEN( range ) );
    Inits.W0.InnerThigh .setZ( Inits.W0.InnerThigh .getZ() * MY_RND_GEN( range ) );
    Inits.W0.InnerShank .setZ( Inits.W0.InnerShank .getZ() * MY_RND_GEN( range ) );
    Inits.W0.InnerFoot  .setZ( Inits.W0.InnerFoot  .getZ() * MY_RND_GEN( range ) );
    Inits.W0.OuterThigh .setZ( Inits.W0.OuterThigh .getZ() * MY_RND_GEN( range ) );
    Inits.W0.OuterShank .setZ( Inits.W0.OuterShank .getZ() * MY_RND_GEN( range ) );

    #undef MY_RND_GEN
}

/////////////////////////////////////////////////////////////////////////////////////////

// Configures the construction info and the initial conditions according to
// specified test suite id.
//
void PassiveWalker::SetupInitialConditions( int configId )
{
    switch( configId )
    {
        case 1:
        case 2:
        case 3:
        case 4:
            Inits.WalkDurationGoal      = btScalar( 8.0 );
            Parameters.InclinationAngle = btScalar( 0.12 ); // radians
            Parameters.PlaneSize        = 200;
            Parameters.FootWidth        = 5;
            Parameters.FootOffsetY      = 12;
            Parameters.FootOffsetX      = 0;
            Parameters.KneeForwardAngle = 10; // degrees

            Inits.V0.HipAxis    .setValue( 0, 0, btScalar( 10.946 ) );
            Inits.V0.InnerThigh .setValue( 0, 0, btScalar(  3.245 ) );
            Inits.V0.InnerShank .setValue( 0, 0, btScalar(  1.492 ) );
            Inits.V0.InnerFoot  .setValue( 0, 0, btScalar(  6.468 ) );
            Inits.V0.OuterThigh .setValue( 0, 0, btScalar(  0.034 ) );
            Inits.V0.OuterShank .setValue( 0, 0, btScalar(  0.012 ) );

            LoadInitialConditions( configId );
            break;

        case 5:
            Inits.WalkDurationGoal      = btScalar( 5.0 );
            Parameters.InclinationAngle = btRadians( btScalar( 6.5 ) ); 
            Parameters.PlaneSize        = 40;
            Parameters.FootWidth        = 5;
            Parameters.FootOffsetY      = 12;
            Parameters.FootOffsetX      = 0;
            Parameters.KneeForwardAngle = 10; // degrees
            Parameters.DisturbFixup     = true;

            Inits.V0.HipAxis    .setValue( 0, 0, btScalar( 12  ) ); 
            Inits.V0.InnerThigh .setValue( 0, 0, btScalar( 2   ) );
            Inits.V0.InnerShank .setValue( 0, 0, btScalar( 3   ) );
            Inits.V0.InnerFoot  .setValue( 0, 0, btScalar( 6   ) );
            Inits.V0.OuterThigh .setValue( 0, 0, btScalar( 0.2 ) );
            Inits.V0.OuterShank .setValue( 0, 0, btScalar( 0.1 ) );

            LoadInitialConditions( configId );
            break;

        case 6:
            Inits.WalkDurationGoal      = btScalar( 5.0 );
            Parameters.InclinationAngle = btRadians( btScalar( 7.5 ) );
            Parameters.PlaneSize        = 40;
            Parameters.FootWidth        = 5;
            Parameters.FootOffsetY      = 12;
            Parameters.FootOffsetX      = 0;
            Parameters.KneeForwardAngle = 10; // degrees

            Inits.V0.HipAxis    .setValue( 0, 0, btScalar( 12.215 ) );
            Inits.V0.InnerThigh .setValue( 0, 0, btScalar(  2.559 ) );
            Inits.V0.InnerShank .setValue( 0, 0, btScalar(  2.236 ) );
            Inits.V0.InnerFoot  .setValue( 0, 0, btScalar(  6.100 ) );
            Inits.V0.OuterThigh .setValue( 0, 0, btScalar(  0.059 ) );
            Inits.V0.OuterShank .setValue( 0, 0, btScalar(  0.053 ) );

            LoadInitialConditions( configId );
            break;

        case 7:
            Inits.WalkDurationGoal      = btScalar( 5.0 );
            Parameters.InclinationAngle = btRadians( btScalar( 8.0 ) );
            Parameters.PlaneSize        = 40;
            Parameters.FootWidth        = 5;
            Parameters.FootOffsetY      = 10;
            Parameters.FootOffsetX      = 0;
            Parameters.KneeForwardAngle = 18; // degrees

            Inits.V0.HipAxis    .setValue( 0, 0, btScalar( 12  ) );
            Inits.V0.InnerThigh .setValue( 0, 0, btScalar( 2   ) );
            Inits.V0.InnerShank .setValue( 0, 0, btScalar( 2.9 ) );
            Inits.V0.InnerFoot  .setValue( 0, 0, btScalar( 6   ) );
            Inits.V0.OuterThigh .setValue( 0, 0, btScalar( 0.2 ) );
            Inits.V0.OuterShank .setValue( 0, 0, btScalar( 0.1 ) );

            LoadInitialConditions( configId );
            break;

        case 8: // Active walker
            Parameters.PlaneSize        = 1000;
            Parameters.HipMass          = btScalar( 2.0 );
            Parameters.ThighMass        = btScalar( 2.0 );
            Parameters.ShankMass        = btScalar( 1.0 );
            Parameters.FootMass         = btScalar( 0.5 );
            Parameters.ShankLength      = 286 - 20;
            Parameters.FootWidth        = 15;
            Parameters.FootLength       = 50;
            Parameters.FootRadius       = 5; // = height
            Parameters.FootOffsetZ      = 10; // 40;
            Parameters.FootOffsetX      = 0;
            Parameters.FootOffsetY      = 15;
            Parameters.InnerLegOffset   = 80;
            Parameters.InnerThighAngle  = 0;
            Parameters.InnerShankAngle  = 0;
            Parameters.OuterThighAngle  = 0;
            Parameters.OuterShankAngle  = 0;
            Parameters.RigidKnees       = true; // initially
            Parameters.KneeForwardAngle = 0; // degrees
            Parameters.FootRestitution  = btScalar( 0.0 );
            Parameters.FootFriction     = btScalar( 5.0 );
            // Active walker parameters:
            Parameters.HipAngleLimit       = btRadians(  50 ); // in radians, 35-50
            Parameters.LegAngularVelocity  = btRadians( 110 ); // in radians, 60-110 
            Parameters.DirectionDiff       = 0;
            State = RECOVERING;
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Configures the walker's angular motors to achieve the current target angles.
//
void PassiveWalker::SetupAngularMotors ()
{
    #if 0
    printf( "%s --- %g %g %g %g\n", GetStateVerbose (),
        TargetHipAngle, TargetInnerKneeAngle, TargetOuterKneeAngle, TargetFootAngle
    );
    #endif

    btScalar deltaL = Parameters.DirectionDiff < 0 ?  Parameters.DirectionDiff : 0;
    btScalar deltaR = Parameters.DirectionDiff > 0 ? -Parameters.DirectionDiff : 0;

    // Hip axis to inner legs

    Inner .Left  .Hip  .SetAngularMotor( 0, TargetHipAngle + deltaL );
    Inner .Right .Hip  .SetAngularMotor( 0, TargetHipAngle + deltaR );

    // Knees

    Inner .Left  .Knee .SetAngularMotor( 0, TargetInnerKneeAngle + deltaL );
    Inner .Right .Knee .SetAngularMotor( 0, TargetInnerKneeAngle + deltaR );
    Outer .Left  .Knee .SetAngularMotor( 0, TargetOuterKneeAngle + deltaL );
    Outer .Right .Knee .SetAngularMotor( 0, TargetOuterKneeAngle + deltaR );

    // Foot ankles

    btScalar innerAnkleAng = -TargetInnerKneeAngle + TargetHipAngle/2 + TargetFootAngle;
    btScalar outerAnkleAng = -TargetOuterKneeAngle - TargetHipAngle/2 + TargetFootAngle;
    btClamp( innerAnkleAng, -SIMD_PI/6, SIMD_PI/3 );
    btClamp( outerAnkleAng, -SIMD_PI/6, SIMD_PI/3 );

    Inner .Left  .Ankle .SetAngularMotor( 0, innerAnkleAng );
    Inner .Right .Ankle .SetAngularMotor( 0, innerAnkleAng );
    Outer .Left  .Ankle .SetAngularMotor( 0, outerAnkleAng );
    Outer .Right .Ankle .SetAngularMotor( 0, outerAnkleAng );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Execute a transition for the walker's finite-state machine.
//
void PassiveWalker::StateMachine( double localTime, btScalar timeStep )
{
    if ( IsDisabledSimulation () ) {
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////

    // Determine the angle between the walker's inner/outer leg pairs
    //
    btScalar leftHipAngle  = Inner.Left .Hip->getAngle(0) - Outer.Left .Hip->getAngle(0);
    btScalar rightHipAngle = Inner.Right.Hip->getAngle(0) - Outer.Right.Hip->getAngle(0);
    HipAngle = ( leftHipAngle + rightHipAngle ) / 2;

    #if 0 // Data collection
    printf( "%lg;%g;%g;%g;%g;%g;\n", localTime, HipAngle, 
        ( Inner.Left.Knee ->getAngle(0) + Inner.Right.Knee ->getAngle(0) ) / 2, 
        ( Inner.Left.Ankle->getAngle(0) + Inner.Right.Ankle->getAngle(0) ) / 2,
        ( Outer.Left.Knee ->getAngle(0) + Outer.Right.Knee ->getAngle(0) ) / 2, 
        ( Outer.Left.Ankle->getAngle(0) + Outer.Right.Ankle->getAngle(0) ) / 2
    );
    #endif

    /////////////////////////////////////////////////////////////////////////////////////

    // Disable the walker who has fallen off the plane.
    //
    btScalar hipHeight = HipAxis->getCenterOfMassPosition().y();
    if ( hipHeight < Parameters.PlaneOffsetY - btScalar( 10 ) )
    {
        DisableSimulation ();
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////

    if ( State == PASSIVE )
    {
        return;
    }
    else if ( State == RECOVERING )
    {
        // Initialize the first stride
        //
        State = INNER_LEG_SWING;
        Parameters.ClampMotorParameters ();
        MotorParametersVerified = true;
        HipAngleLimit        = Parameters.HipAngleLimit;
        LegAngularVelocity   = Parameters.LegAngularVelocity;
        DirectionDiff        = Parameters.DirectionDiff;
        TargetHipAngle       = 0;
        TargetInnerKneeAngle = 0;
        TargetOuterKneeAngle = 0;
        TargetFootAngle      = 0;
        return;
    }
    else if ( HipGroundContact  )
    {
        State = PASSIVE;
        TargetHipAngle       = 0;
        TargetInnerKneeAngle = 0;
        TargetOuterKneeAngle = 0;
        TargetFootAngle      = 0; 
        SetupAngularMotors ();
        return;
    }

    /////////////////////////////////////////////////////////////////////////////////////

    btScalar angVelSlower = LegAngularVelocity * timeStep;
    btScalar angVelFaster = 2 * LegAngularVelocity * timeStep;

    // A single stride consists of the following phases:
    //
    //   - inner leg swing
    //   - inner leg stance
    //   - outer leg swing
    //   - outer leg stance
    //
    if ( State == INNER_LEG_SWING )
    {
        if ( TargetHipAngle < HipAngleLimit ) {
            TargetHipAngle += angVelSlower;
            TargetInnerKneeAngle += angVelFaster;
            TargetOuterKneeAngle -= angVelSlower;
        } 
        else {
            State = INNER_LEG_STANCE;
        }
    }
    else if ( State == INNER_LEG_STANCE )
    {
        if ( TargetHipAngle > 0 ) {
            TargetHipAngle -= angVelSlower;
            TargetInnerKneeAngle -= angVelFaster;
        }
        else if ( TargetOuterKneeAngle < 0 ) {
            TargetOuterKneeAngle += angVelSlower;
        }
        else {
            State = OUTER_LEG_SWING;
            Parameters.ClampMotorParameters ();
            MotorParametersVerified = true;
            HipAngleLimit        = Parameters.HipAngleLimit;
            LegAngularVelocity   = Parameters.LegAngularVelocity;
            DirectionDiff        = Parameters.DirectionDiff;
            TargetHipAngle       = 0;
            TargetInnerKneeAngle = 0;
            TargetOuterKneeAngle = 0;
            TargetFootAngle      = 0; 
        }
    }
    else if ( State == OUTER_LEG_SWING )
    {
        if ( TargetHipAngle > -HipAngleLimit ) {
            TargetHipAngle -= angVelSlower;
            TargetOuterKneeAngle += angVelFaster;
            TargetInnerKneeAngle -= angVelSlower;
        } 
        else {
            State = OUTER_LEG_STANCE;
        }
    }
    else if ( State == OUTER_LEG_STANCE )
    {
        if ( TargetHipAngle < 0 ) {
            TargetHipAngle += angVelSlower;
            TargetOuterKneeAngle -= angVelFaster;
        }
        else if ( TargetInnerKneeAngle < 0 ) {
            TargetInnerKneeAngle += angVelSlower;
        }
        else {
            State = INNER_LEG_SWING;
            Parameters.ClampMotorParameters ();
            MotorParametersVerified = true;
            HipAngleLimit        = Parameters.HipAngleLimit;
            LegAngularVelocity   = Parameters.LegAngularVelocity;
            DirectionDiff        = Parameters.DirectionDiff;
            TargetHipAngle       = 0;
            TargetInnerKneeAngle = 0;
            TargetOuterKneeAngle = 0;
            TargetFootAngle      = 0; 
        }
    }

    SetupAngularMotors ();
}
