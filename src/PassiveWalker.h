#pragma once
#ifndef _PASSIVE_WALKER_H_INCLUDED
#define _PASSIVE_WALKER_H_INCLUDED

/**
 *  @file      PassiveWalker.h
 *  @brief     Definitions for the PassiveWalker class that encapsulates
 *             a simple passive walker with knees.
 *  @author    Mikica Kocic
 *  @version   0.16
 *  @date      2012-06-16
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Machine.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Referenced BT classes

// #include "BulletDynamics/Dynamics/btActionInterface.h"
// #include "LinearMath/btIDebugDraw.h"

class btActionInterface;

/////////////////////////////////////////////////////////////////////////////////////////

class PassiveWalker : public Machine
{
    /** Represents a set of states for the walker's finite-state machine.
     * Note: When changed must be synchronized with PassiveWalker::GetStateVerbose().
     */
    enum State
    {
        UNINITIALIZED = 0,
        PASSIVE,
        RECOVERING,
        INNER_LEG_SWING,
        INNER_LEG_STANCE,
        OUTER_LEG_SWING,
        OUTER_LEG_STANCE
    };

    /** Represents a passive walker's leg assembly consisting of a thigh, shank and foot
     * as machine elements together with a hip, knee and ankle as joints, and a foot
     * ground plane contact sensor.
     */
    struct LegAssembly
    {
        Machine::Element Thigh;
        Machine::Element Shank;
        Machine::Element Foot;
        Machine::Joint  Hip;
        Machine::Joint  Knee;
        Machine::Joint  Ankle;
        Machine::ContactSensor FootGroundContact;
    };

    /** Represents a pair of leg assembly (inner or outer) of the walker consisting of
     * left and right lag assembly and thigh and shank locks (optional) between them.
     */
    struct LegPairAssembly
    {
        LegAssembly    Left;
        LegAssembly    Right;
        Machine::Joint ThighLock;
        Machine::Joint ShankLock;
    };

    /** Holds the state for the walker's finite-state machine. 
     */
    State State;

    /** Holds the hip axis that connects upper thighs of the leg assemblies.
     */
    Machine::Element HipAxis;

    /** Holds the inner leg pair assembly for the walker.
     */
    LegPairAssembly Inner;

    /** Holds the outer leg pair assembly for the walker.
     */
    LegPairAssembly Outer;

    /** Holds a contact sensor for collisions between the hip axis and the ground plane.
     */
    Machine::ContactSensor HipGroundContact;

    /** Holds the actual angle between inner and outer leg assemblies.
     */
    btScalar HipAngle;

    /** Holds the target hip angle for the hip motor.
     */
    btScalar TargetHipAngle;

    /** Holds the target inner knee angle for the inner knee motors.
     */
    btScalar TargetInnerKneeAngle;

    /** Holds the target inner knee angle for the outer knee motors.
     */
    btScalar TargetOuterKneeAngle;

    /** Holds the target foot angle for the feet motors.
     */
    btScalar TargetFootAngle;

    /** Holds the maximum allowed hip angle for the next stride.
     */
    btScalar HipAngleLimit;

    /** Holds the reference angular velocity for the leg assemblies for the next stride.
     */
    btScalar LegAngularVelocity;

    /** Hold the direction difference (between the left/right motor angles).
     */
    btScalar DirectionDiff;

    /** Indicates whether the motor parameters are verififed as OK.
        */
    bool MotorParametersVerified;

    friend class PassiveWalkerTestBed;
    friend class PassiveWalkerApplication;

public:

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents initial conditions (linear and angular velocities) for a walker.
     */
    struct InitialConditions
    {
        struct Velocities
        {
            btVector3 HipAxis;
            btVector3 InnerThigh;
            btVector3 InnerShank;
            btVector3 InnerFoot;
            btVector3 OuterThigh;
            btVector3 OuterShank;
            btVector3 OuterFoot;
        };

        /** Holds the initial linear velocities for a walker's machine elements.
         */
        Velocities V0;

        /** Holds the initial angular velocities for a walker's machine elements.
         */
        Velocities W0;

        /** Holds a minimum walk duration goal found for this set of initial conditions.
         */
        btScalar WalkDurationGoal;

        /** Constructs a default initial conditions (all zero's) 
         */
        InitialConditions ()
        {
            V0.HipAxis    .setZero ();
            V0.InnerThigh .setZero ();
            V0.InnerShank .setZero ();
            V0.InnerFoot  .setZero ();
            V0.OuterThigh .setZero ();
            V0.OuterShank .setZero ();
            V0.OuterFoot  .setZero ();

            W0.HipAxis    .setZero ();
            W0.InnerThigh .setZero ();
            W0.InnerShank .setZero ();
            W0.InnerFoot  .setZero ();
            W0.OuterThigh .setZero ();
            W0.OuterShank .setZero ();
            W0.OuterFoot  .setZero ();

            WalkDurationGoal = 0;
        }

        /** Displays the contents of the structure on stdout.
         */
        void Dump () const;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents mechanical parameters for a walker and its test bed.
     */
    struct ConstructionInfo
    {
        /** Holds the mass of the walker's hip.
         */
        btScalar HipMass;

        /** Holds the mass of the walker's thigh.
         */
        btScalar ThighMass;

        /** Holds the mass of the walker's shank.
         */
        btScalar ShankMass;

        /** Holds the mass of the walker's foot.
         */
        btScalar FootMass;
    

        /** Holds the length dimensions multiplier (units and scale multiplier).
         * E.g. if leg radius is 100 mm and a walker is scaled 10x then
         * the length units should be 10 * 10^-3.
         */
        btScalar LengthUnits;

        /** Holds the 'radius' of the walker's legs. Actuall legs are boxes and
         * the radius is in fact a half-size of the square cross section.
         */
        btScalar LegRadius;

        /** Holds the offset of the inner leg assembly from the vertical main axis.
         */
        btScalar InnerLegOffset;

        /** Holds the offset of the outer leg assembly from the walker's sagittal plane
         */
        btScalar OuterLegOffset;

        /** Holds the length of the walker's thigh.
         */
        btScalar ThighLength;

        /** Holds the length of the walker's shank.
         */
        btScalar ShankLength;

        /** Holds the radius of the walker's foot if the foot is cylinder.
         * If the foot is cuboid, foot radius represents the cuboid's height half-size.
         */
        btScalar FootRadius;

        /** Holds the half-size width of the walker's foot.
         */
        btScalar FootWidth;

        /** Holds the half-size length of the walker's foot. It may be 0, in which
         * case the foot is a cylinder (otherwise it is a cuboid).
         */
        btScalar FootLength;

        /** Holds the horizontal frontal offset from the walker's coronal reference plane.
         */
        btScalar FootOffsetZ;

        /** Holds the vertial offset between the walker's foot and its shank.
         */
        btScalar FootOffsetY;

        /** Holds the horizontal offset from the walker's sagittal reference plane.
         */
        btScalar FootOffsetX;

        /** Holds the maximum forward (allowed depression) angle of the walker knee.
         * The maximum backward knee angle is pi.
         */
        btScalar KneeForwardAngle;

        /** Indicates whether the walker's knees are rigid (locked).
         */
        bool RigidKnees;

        /** Indicates whether the walker's outer leg pairs are 'welded' to the hip axis.
         */
        bool LockOuterLegsToHipAxis;

        /** Holds the initial angle between the inner thigh and the coronal plane.
         */
        btScalar InnerThighAngle;

        /** Holds the initial angle between the inner shank and the coronal plane.
         */
        btScalar InnerShankAngle;

        /** Holds the initial angle between the outer thigh and the coronal plane.
         */
        btScalar OuterThighAngle;

        /** Holds the initial angle between the outer shank and the coronal plane.
         */
        btScalar OuterShankAngle;

        /** Holds the initial position of the hip axis center of mass.
         */
        btVector3 Position;

        /** Holds the initial orientation of the walker, relative to the hip axis cm.
         */
        btVector3 EulerZYX;

        /** Indicates whether to disturb inital condions with a random error.
         */
        bool DisturbFixup;

        /** Holds the friction coefficient between the walker's feet and the ground.
         */
        btScalar FootFriction;

        /** Holds the coefficient of restitution between the walker's feet and the ground.
         */
        btScalar FootRestitution;

        /** Holds the maximum allowed hip angle (if active walker).
         */
        btScalar HipAngleLimit;

        /** Holds the reference angular velocity for the leg assemblies (active walker).
         */
        btScalar LegAngularVelocity;

        /** Hold the direction difference (between the left/right motor angles).
         */
        btScalar DirectionDiff;

        /** Holds the initial inclination angle for the ground plane.
         */
        btScalar InclinationAngle;

        /** Holds the required ground plane size.
         */
        btScalar PlaneSize;

        /** Holds the required ground plane thickness.
         */
        btScalar PlaneThickness;

        /** Holds the ground plane vertical offset from the coordinate origin.
         */
        btScalar PlaneOffsetY;

        /** Constructs a set of parameters for a lab5 template passive walker.
         */
        ConstructionInfo ()
        {
            HipMass   = btScalar( 0.50 );  // kg
            ThighMass = btScalar( 2.50 );  // kg
            ShankMass = btScalar( 1.00 );  // kg
            FootMass  = btScalar( 0.50 );  // kg

            LengthUnits = btScalar( 10 * 0.001 ); // Dimensions are in mm scaled 10x

            LegRadius      =  20;  // mm
            InnerLegOffset =  44;  // mm
            OuterLegOffset = 132;  // mm
            ThighLength    = 286;  // mm
            ShankLength    = 216;  // mm
            FootRadius     =  87;  // mm
            FootWidth      =  10;  // mm
            FootLength     =   0;  // mm
            FootOffsetZ    =  55;  // mm
            FootOffsetY    =  12;  // mm
            FootOffsetX    =  15;  // mm

            KneeForwardAngle = 0;  // degrees
            RigidKnees = false;

            LockOuterLegsToHipAxis = true;

            InnerThighAngle = 0;   // degrees
            InnerShankAngle = 0;   // degrees
            OuterThighAngle = 0;   // degrees
            OuterShankAngle = 0;   // degrees

            Position.setZero ();
            EulerZYX.setZero ();

            FootFriction    = btScalar( 1.0 );
            FootRestitution = btScalar( 0.8 );

            // Active walker motor parameters
            // 
            HipAngleLimit       = btRadians(  50 ); // in radians; range [ 35;  50 ] deg
            LegAngularVelocity  = btRadians( 110 ); // in radians; range [ 60; 110 ] deg
            DirectionDiff       = 0;                // in radians; range [ -5;  +5 ] deg

            // Ground plane
            //
            InclinationAngle =  0;  // degrees
            PlaneThickness   =  1;  // meters
            PlaneSize        = 80;  // meters
            PlaneOffsetY     = -2;  // meters
        }

        /** Ensures the accepted range of values for the motor parameters
         */
        void ClampMotorParameters ()
        {
            btClamp( HipAngleLimit,      btRadians( 20 ), btRadians(  60 ) );
            btClamp( LegAngularVelocity, btRadians( 40 ), btRadians( 150 ) );
            btClamp( DirectionDiff,      btRadians( -5 ), btRadians(  +5 ) );
        }
    };

    /** Holds the construction info for this walker.
     */
    ConstructionInfo Parameters;

    /** Holds the initial conditions (the motion state variables) for this walker.
     */
    InitialConditions Inits;

    /////////////////////////////////////////////////////////////////////////////////////

public:

    /** Constructs a new uninitialized passive walker in the given dynamics world.
     */
    PassiveWalker( btDynamicsWorld* ownerWorld )
        : Machine( ownerWorld )
        , State( UNINITIALIZED )
        , MotorParametersVerified( false )
    {
    }

    /** Creates a set of machine elements and joints for the walker according
     * to the given construction info found in PassiveWalker::Parameters and
     * with the initial conditions found in PassiveWalker::Inits.
     */
    void Create ();

    /** Displays the walker's motion state variables on stdout.
     */
    void Dump( double localTime ) const;

    /** Returns ture if the walker is disabled for simulation.
     */
    bool IsDisabledSimulation () const
    {
        return HipAxis.HasRigidBody ()
            && HipAxis->getActivationState () == DISABLE_SIMULATION;
    }

    /** Returns if the walker is motor driven.
     */
    bool IsAutonomous () const
    {
        return State != PASSIVE && State != UNINITIALIZED;
    }

    /** Gets the verbose state of the finite-state machine.
     */
    const char* GetStateVerbose () const
    {
        static const char* verboseStates [] = {
            /* UNINITIALIZED = 0 */  "Uninitialized",
            /* PASSIVE           */  "Passive",
            /* RECOVERING        */  "Recovering",
            /* INNER_LEG_SWING   */  "Inner-leg Swing",
            /* INNER_LEG_STANCE  */  "Inner-leg Stance",
            /* OUTER_LEG_SWING   */  "Outer-leg Swing",
            /* OUTER_LEG_STANCE  */  "Outer-leg Stance"
        };
        return verboseStates[ State ];
    }

    /** Configures the construction info and the initial conditions according to the
     * specified test suite id.
     */
    void SetupInitialConditions( int configId );

    /** Loads initial conditions from file 'config-[id].txt'.
     */
    void LoadInitialConditions( int configId );

    /** Randomizes the initial conditions uniformly 10% around the given center values.
     */
    void RandomizeInitialConditions ();

    /** Saves the initial conditions into a file 'config-[id].txt'.
     */
    void SaveInitialConditions( int configId, double localTime ) const;

    /** Connects the walker's contact sensors to the specified ground plane object.
     */
    void SetupGroundPlaneSensors( Machine::Element& GroundPlane );

    /** Execute a transition for the walker's finite-state machine.
     * Should be called on every pre-tick.
     */
    void StateMachine( double localTime, btScalar timeStep );

    /** Configures the walker's angular motors to achieve the current target angles.
     */
    void SetupAngularMotors ();

    /** Returns the center of mass of the walker.
     */
    btVector3 GetCenterOfMass () const;
};

#endif // _PASSIVE_WALKER_H_INCLUDED
