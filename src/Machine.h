#pragma once
#ifndef _MACHINE_H_INCLUDED
#define _MACHINE_H_INCLUDED

/**
 *  @file      Machine.h
 *  @brief     Definitions for the Machine class that represents a collection of
 *             rigid bodies (machine elements) with constraints (joints).
 *  @author    Mikica Kocic
 *  @version   0.5
 *  @date      2012-06-15
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Utilities.h"
#include "Constants.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Referenced BT classes

#include "btBulletDynamicsCommon.h"

class  btCollisionWorld;
class  btCollisionShape;
class  btRigidBody;
class  btGeneric6DofConstraint;
struct btDefaultMotionState;

/////////////////////////////////////////////////////////////////////////////////////////

/** Represents a collection of machine elements (rigid bodies) and joints (constraints).
 */
class Machine
{
public:

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a machine element that encapsulates a rigid body with a shape.
     */
    class Element
    {
        /** Holds the owner machine of this element.
         */
        Machine* Owner;

        /** Indicates whether the object is automatially deleted when removed from
         * the owner machine's element collection.
         */
        bool AutoDelete;

        /** Holds an instance of a collision shape associated to this element.
         */
        btCollisionShape* Shape;

        /** Holds an instance of a rigid body associated to this machine element.
         */
        btRigidBody* RigidBody;

        /** Holds a common implementation to synchronize world transforms with offsets.
         * Cotnains center of mass offset, start world transform and graphics
         * world transform.
         */
        btDefaultMotionState MotionState;

        friend class Machine;
        friend class Joint;

    private:

        /** Connects this element to an existing machine assembly.
         */
        void ConnectTo( Machine* machineAssembly )
        {
            Owner = machineAssembly;
            Owner->Elements.push_back( this );

            if ( Owner && Owner->World ) {
                Owner->World->addRigidBody( RigidBody );
            }
        }

	    BT_DECLARE_ALIGNED_ALLOCATOR ();

    public:

        /** Allocates a new element marked to be automatically deleted on removal
         * from the owner machine's element collection.
         * Note that operator new() is declared as private and not accessible
         * to the user.
         */
        static Element* New ()
        {
            Element* instance = new Element ();
            instance->AutoDelete = true;
            return instance;
        }

        /** Constructs a disconnected (detached) machine element.
         */
        Element ()
            : Owner( 0 )
            , AutoDelete( false )
            , Shape( 0 )
            , RigidBody( 0 )
        {
        }

        /** Destructs (disconnets) this element from the owner (if any).
         */
        ~Element ()
        {
            Disconnect ();
        }

        /** Disconnects this element from the owner machine's assembly.
         */
        void Disconnect ()
        {
            if ( Owner && Owner->World && RigidBody ) {
                Owner->World->removeRigidBody( RigidBody );
            }

            if ( RigidBody ) {
                delete RigidBody;
                RigidBody = 0;
            }

            if ( Shape ) {
                delete Shape;
                Shape = 0;
            }


            if ( Owner ) {
                Owner->Elements.remove( this );
                Owner = 0;
            }
        }

        /** Returns true if the element has an instantiated rigid body.
         */
        bool HasRigidBody () const
        {
            return RigidBody != 0;
        }

        /** Provides accesse to the methods and properties of the underlying rigid body.
         */
        btRigidBody* operator -> ()
        {
            return RigidBody;
        }

        /** Provides access to the constant methods and properties of the rigid body.
         */
        const btRigidBody* operator -> () const
        {
            return RigidBody;
        }

        /** Enables continuous collision detection for a given characteristic length.
         */
        void EnableCCD( btScalar length )
        {
            if ( RigidBody ) {
                RigidBody->setCcdMotionThreshold( length  );
                RigidBody->setCcdSweptSphereRadius( btScalar( 0.2 ) * length );
            }
        }

        /** Creates a rigid body of a capsule shape, and connects the element to 
         * the given machine assembly.
         */
        void CapsuleShape(
            Machine* machineAssembly,
            btScalar mass, 
            const btTransform& startTransform, 
            const btVector3& relativePosition,
            const btVector3& orientation,
            btScalar radius,
            btScalar height,
            btScalar scale = btScalar( 1 )
            )
        {
            Create( machineAssembly, 
                new btCapsuleShape( radius * scale, height * scale ), 
                mass, startTransform, relativePosition * scale, orientation
            );

            // EnableCCD( scale * ( radius < height ? radius : height ) );
        }

        /** Creates a rigid body of a box (cuboid) shape, and connects the element to 
         * the given machine assembly.
         */
        void BoxShape(
            Machine* machineAssembly,
            btScalar mass, 
            const btTransform& startTransform, 
            const btVector3& relativePosition,
            const btVector3& orientation,
            const btVector3& halfExtents,
            btScalar scale = btScalar( 1 )
            )
        {
            Create( machineAssembly, 
                new btBoxShape( halfExtents * scale ),
                mass, startTransform, relativePosition * scale, orientation
            );

            // EnableCCD( scale * halfExtents.minAxis () );
        }

        /** Creates a rigid body of a cylinder shape, and connects the element to 
         * the given machine assembly.
         */
        void CylinderShape(
            Machine* machineAssembly,
            btScalar mass, 
            const btTransform& startTransform, 
            const btVector3& relativePosition,
            const btVector3& orientation,
            const btVector3& halfExtents,
            btScalar scale = btScalar( 1 )
            )
        {
            Create( machineAssembly, 
                new btCylinderShape( halfExtents * scale ),
                mass, startTransform, relativePosition * scale, orientation
            );

            // EnableCCD( scale * halfExtents.minAxis () );
        }

        /** Creates a static plane rigid body, and connects the element to 
         * the given machine assembly.
         */
        void PlaneShape(
            Machine* machineAssembly,
            const btVector3& relativePosition,
            const btVector3& orientation
            )
        {
            Create( machineAssembly, 
                new btStaticPlaneShape( Const::Y, 0 ),
                0, btTransform::getIdentity (), relativePosition, orientation
            );
        }

        /** Creates a rigid body for the given shape, and connects the element to 
         * the given machine assembly.
         */
        void Create(
            Machine* machineAssembly,
            btCollisionShape* shape,
            btScalar mass, 
            const btTransform& frameOfReference, 
            const btVector3& position,
            const btVector3& orientation
            )
        {
            Disconnect ();

            Shape = shape;

            MotionState.setWorldTransform( 
                frameOfReference 
                * Transform( position ).setEulerZYX( orientation )
            );

            btVector3 localInertia( 0, 0, 0 );
            if ( mass != 0 ) {  // i.e. if it is dynamic
                Shape->calculateLocalInertia( mass, localInertia );
            }

            RigidBody = new btRigidBody( mass, &MotionState, Shape, localInertia );

            // Setup some damping on the rigid body
            //
            RigidBody->setDamping( btScalar( 0.05 ), btScalar( 0.1 ) );

            ConnectTo( machineAssembly );
        }

        /** Sets mass and local inertia of the rigid body.
         */
        void SetMassProperties( btScalar mass )
        {
            if ( Shape && RigidBody )
            {
                btVector3 localInertia( 0, 0, 0 );

                if ( mass != 0 ) {  // i.e. if it's dynamic
                    Shape->calculateLocalInertia( mass, localInertia );
                }

                RigidBody->setMassProps( mass, localInertia );
            }
        }

        /** Enables the underlying rigid body to be deactivated.
         */
        void EnableDeactivation ()
        {
            if ( RigidBody ) {
                RigidBody->forceActivationState( ACTIVE_TAG );
                RigidBody->setDeactivationTime( 0.8f );
                RigidBody->setSleepingThresholds( 1.6f, 2.5f );
            }
        }

        /** Disables deactivation for the underlying rigid body.
         */
        void DisableDeactivation ()
        {
            if ( RigidBody ) {
                RigidBody->forceActivationState( DISABLE_DEACTIVATION );
            }
        }

        /** Disables simulation for the underlying rigid body.
         */
        void DisableSimulation ()
        {
            if ( RigidBody ) {
                RigidBody->forceActivationState( DISABLE_SIMULATION );
            }
        }

        /** Sets the linear velocity of the underlying rigid body.
         */
        void SetLinearVelocity( btScalar vx, btScalar vy, btScalar vz )
        {
            if ( RigidBody ) {
                RigidBody->setLinearVelocity( btVector3( vx, vy, vz ) );
            }
        }

        /** Sets the linear velocity of the underlying rigid body.
         */
        void SetLinearVelocity( const btVector3& velocity )
        {
            if ( RigidBody ) {
                RigidBody->setLinearVelocity( velocity );
            }
        }

        /** Sets the angular velocity of the underlying rigid body.
         */
        void SetAngularVelocity( btScalar wx, btScalar wy, btScalar wz )
        {
            if ( RigidBody ) {
                RigidBody->setAngularVelocity( btVector3( wx, wy, wz ) );
            }
        }

        /** Sets the angular velocity of the underlying rigid body.
         */
        void SetAngularVelocity( const btVector3& velocity )
        {
            if ( RigidBody ) {
                RigidBody->setAngularVelocity( velocity );
            }
        }

        /** Gets one of primary axes of the underlying rigid body.
         */
        btVector3 GetAxis( int axis )
        {
            if ( RigidBody ) {
                return RigidBody->getWorldTransform ().getBasis().getColumn( axis );
            }
            return Const::Zero;
        }

        /** Displays the motion state of the underlying rigid body.
         */
        void Dump( const char* title ) const;
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a joint between machine elements (constraint betwen rigid bodies).
     */
    class Joint
    {
        /** Holds the owner of this machine element.
         */
        Machine* Owner;

        /** Indicates whether the object is automatially deleted when removed from
         * the owner machine's element collection.
         */
        bool AutoDelete;

        /** Holds a 6DOF constraint between two rigidbodies each with a pivotpoint 
         * that descibes the axis location in local space.
         */
        btGeneric6DofConstraint* Constraint;

        friend class Machine;

    private:

        /** Connects this element to a machine assembly.
         */
        void ConnectTo( Machine* machineAssembly )
        {
            Owner = machineAssembly;
            Owner->Joints.push_back( this );

            if ( Owner && Owner->World ) {
                Owner->World->addConstraint( Constraint, true );
            }
        }

	    BT_DECLARE_ALIGNED_ALLOCATOR ();

    public:

        /** Allocates a new joint marked to be automatically deleted on removal
         * from the owner machine's joint collection.
         * Note that operator new() is declared as private and not accessible
         * to the user.
         */
        static Joint* New ()
        {
            Joint* instance = new Joint ();
            instance->AutoDelete = true;
            return instance;
        }

        /** Constructs a disconnected (detached) machine joint.
         */
        Joint ()
            : Owner( 0 )
            , AutoDelete( false )
            , Constraint( 0 )
        {
        }

        /** Destructs (disconnets) this joint from the owner machine (if any).
         */
        ~Joint ()
        {
            Disconnect ();
        }

        /** Disconnects this joint from the owner machine's assembly.
         */
        void Disconnect ()
        {
            if ( Owner && Owner->World && Constraint ) {
                Owner->World->removeConstraint( Constraint );
            }

            if ( Constraint ) {
                delete Constraint;
                Constraint = 0;
            }

            if ( Owner ) {
                Owner->Joints.remove( this );
                Owner = 0;
            }
        }

        /** Provides access to the members of the underlying 6DOF constraint class.
         */
        btGeneric6DofConstraint* operator -> ()
        {
            return Constraint;
        }

        /** Providess access to the constan members of the underlying 6DOF constraint.
         */
        const btGeneric6DofConstraint* operator -> () const
        {
            return Constraint;
        }

        /** Creats a general 6DOF joint between two machine elements.
         */
        void Create6DOF(
            Machine* machineAssembly, 
            const Element& bodyPartA, const btVector3& localPositionA, 
            const Element& bodyPartB, const btVector3& localPositionB,
            const btVector3& orientationB,
            bool rigidJoint,
            const btVector3& angularLowerLimit,
            const btVector3& angularUpperLimit,
            btScalar scale
            )
        {
            Disconnect ();

            Constraint = new btGeneric6DofConstraint(
                *bodyPartA.RigidBody, *bodyPartB.RigidBody, 
                Transform( localPositionA * scale ), 
                Transform( localPositionB * scale ).setEulerZYX( orientationB ),
                /* useLinearReferenceFrame */ true
            );

            Constraint->setDbgDrawSize( btScalar( 5 ) );

            if ( rigidJoint )
            {
                Constraint->setAngularLowerLimit( Const::Zero );
                Constraint->setAngularUpperLimit( Const::Zero );
            }
            else
            {
                Constraint->setAngularLowerLimit( angularLowerLimit );
                Constraint->setAngularUpperLimit( angularUpperLimit );
            }

            Constraint->setLinearLowerLimit( Const::Zero );
            Constraint->setLinearUpperLimit( Const::Zero );

            for ( int i = 0; i < 6; ++i )
            {
                Constraint->setParam( BT_CONSTRAINT_STOP_CFM, 0.0f, i );
                Constraint->setParam( BT_CONSTRAINT_STOP_ERP, 0.9f, i );
            }

            ConnectTo( machineAssembly );
        }

        /** Creats a rigid joint (constraint) between two machine elements.
         */
        void CreateFixed(
            Machine* machineAssembly, 
            const Element& bodyPartA, const btVector3& localPositionA, 
            const Element& bodyPartB, const btVector3& localPositionB,
            const btVector3& orientationB,
            btScalar scale
            )
        {
            Create6DOF( machineAssembly, 
                bodyPartA, localPositionA, 
                bodyPartB, localPositionB, orientationB, 
                /* rigid */ true, Const::Zero, Const::Zero, scale
            );
        }

        /** Sets angular limits for the given axis of the joint.
         */
        btRotationalLimitMotor* SetAngularLimits( 
            int axis, btScalar lowerAngle, btScalar upperAngle
            )
        {
            btRotationalLimitMotor* rlm = Constraint->getRotationalLimitMotor( axis );
            rlm->m_loLimit = btNormalizeAngle( lowerAngle );
            rlm->m_hiLimit = btNormalizeAngle( upperAngle );
            return rlm;
        }

        /** Sets angular motor parameters for the given axis of the joint.
         */
        btRotationalLimitMotor* SetAngularMotor(
            int axis, btScalar targetVelocity,
            btScalar maxMotorForce = btScalar( 300 ),
            bool useLimiter = true
            )
        {
            btRotationalLimitMotor* rlm = useLimiter
                ? SetAngularLimits( axis, targetVelocity, targetVelocity )
                : UnlockAngular( axis );
            rlm->m_enableMotor    = true;
            rlm->m_maxLimitForce  = maxMotorForce;
            rlm->m_maxMotorForce  = maxMotorForce;
            rlm->m_targetVelocity = useLimiter ? btScalar( 0 ) : targetVelocity;
            rlm->m_stopERP        = btScalar( 0.2 ); // Default: 0.2
            rlm->m_normalCFM      = btScalar( 0.0 ); // Default: 0
            rlm->m_stopCFM        = btScalar( 0.0 ); // Default: 0
            rlm->m_bounce         = btScalar( 0.0 ); // Default: 0
            rlm->m_damping        = btScalar( 1.0 ); // Default: 1
            rlm->m_limitSoftness  = btScalar( 0.5 ); // Default: 0.5
            return rlm;
        }

        /** Unlocks angular limits for the given axis of the joint (disables limits).
         */
        btRotationalLimitMotor* UnlockAngular( int axis )
        {
            return SetAngularLimits( axis, +1, -1 );
        }

        /** Locks the joint to a certain angle.
         */
        btRotationalLimitMotor* LockAngular(
            int axis, btScalar angle = 0, btScalar maxLimitForce = btScalar( 300 )
            )
        {
            btRotationalLimitMotor* rlm = SetAngularLimits( axis, angle, angle );
            rlm->m_enableMotor   = false;
            rlm->m_maxLimitForce = maxLimitForce;
            rlm->m_maxMotorForce = maxLimitForce;
            rlm->m_stopERP       = btScalar( 0.2 ); // Default: 0.2
            rlm->m_normalCFM     = btScalar( 0.0 ); // Default: 0
            rlm->m_stopCFM       = btScalar( 0.0 ); // Default: 0
            rlm->m_bounce        = btScalar( 0.0 ); // Default: 0
            rlm->m_damping       = btScalar( 1.0 ); // Default: 1
            rlm->m_limitSoftness = btScalar( 0.5 ); // Default: 0.5
            return rlm;
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents a contact sensor between two machine elements.
     */
    class ContactSensor : public btCollisionWorld::ContactResultCallback
    {
        /** Holds the first machine element for which contact points are detected.
         */
        Machine::Element* Element_A;

        /** Holds the second machine element for which contact points are detected.
         */
        Machine::Element* Element_B;

        /** Holds an indicator whether there was a contact between two elements.
         */
        volatile bool InContact;

    public:

        /** Constrats a contact sensor for the given machine elements.
         * Without parameters creates an empty (unuassociated to any elements) sensor.
         */
        ContactSensor( Element* a = 0, Element* b = 0 )
            : Element_A( 0 )
            , Element_B( 0 )
        {}

        /** Constrats a contact sensor for the given machine elements.
         */
        ContactSensor( Element& a, Element& b ) 
            : Element_A( &a )
            , Element_B( &b )
        {}

        /** Associates a contact sensor for the given machine elements.
         */
        void SetupPair( Element& a, Element& b )
        {
            Element_A = &a;
            Element_B = &b;
        }

        /** Associates a contact sensor for the given machine elements.
         */
        void SetupPair( Element* a, Element* b )
        {
            Element_A = a;
            Element_B = b;
        }

        /** Returns true if there was a contact detected between two rigid bodies.
         */
        operator bool ()
        {
            InContact = false;

            if ( Element_A && Element_A->RigidBody
                && Element_A->Owner && Element_A->Owner->World
                && Element_B && Element_B->RigidBody
                )
            {
                Element_A->Owner->World->contactPairTest( 
                    Element_A->RigidBody, Element_B->RigidBody, *this
                );
            }

            return InContact;
        }

        /** Contact result callback called from an dynamics world instance
         * when there was a contact detected between two rigid bodies.
         */
	    virtual	btScalar addSingleResult( btManifoldPoint& cp,
                const btCollisionObject* colObj0, int partId0, int index0,
                const btCollisionObject* colObj1, int partId1, int index1
            )
        {
            InContact = true;
            return 1;
        }
    };

    /////////////////////////////////////////////////////////////////////////////////////

protected:

    /** Holds the world 
     */
    btDynamicsWorld* World;

    /** Holds a collection of machine elements.
     */
    btAlignedObjectArray<Machine::Element*> Elements;

    /** Holds a collection of joints between machine elements.
     */
    btAlignedObjectArray<Machine::Joint*> Joints;

    friend class Machine::Element;
    friend class Machine::Joint;

protected:

    /** Default constructor that creates a detached instance (unconnected to any 
     * dynamics world). Declared as protected (should not be called by the user.)
     */
    Machine ()
        : World( 0 )
    {
    }

public:

	BT_DECLARE_ALIGNED_ALLOCATOR ();

    /** Constructs an instance with rigid bodies and constraints created in 
     * the given dynamics world.
     */
    Machine( btDynamicsWorld* world )
        : World( world )
    {
        btAssert( World != 0 );
    }

    /** Destructs the machine and removes all belonging machine elements and joints.
     */
    virtual ~Machine ()
    {
        // Remove all joints between machine elements
        //
        for ( int i = Joints.size () - 1; i >= 0; --i )
        {
            Joint* joint = Joints.at(i);
            joint->Disconnect ();
            if ( joint->AutoDelete ) {
                delete joint;
            }
        }

        Joints.clear ();

        // Remove all machine elements
        //
        for ( int i = Elements.size () - 1; i >= 0; --i )
        {
            Element* element = Elements.at(i);
            element->Disconnect ();
            if ( element->AutoDelete ) {
                delete element;
            }
        }

        Elements.clear ();
    }

    /** Returns the associated dynamics world.
     */
    btDynamicsWorld* GetDynamicsWorld ()
    {
        return World;
    }

    /** Sets a debug drawer for objects in the world.
     */
    void SetDebugDrawer( btIDebugDraw* debugDrawer )
    {
        if ( World ) {
            World->setDebugDrawer( debugDrawer );
        }
    }

    /** Calls a debug drawer for the belonging dynamics world.
     */
    void DebugDrawWorld ()
    {
        if ( World ) {
            World->debugDrawWorld ();
        }
    }

    /** Enables deactivation for all belonging machine elements (rigid bodies).
     */
    void EnableDeactivation ()
    {
        for( int i = 0; i < Elements.size (); ++i ) {
            Elements.at(i)->EnableDeactivation ();
        }
    }

    /** Disables deactivation for all belonging machine elements (rigid bodies).
     */
    void DisableDeactivation ()
    {
        for( int i = 0; i < Elements.size (); ++i ) {
            Elements.at(i)->DisableDeactivation ();
        }
    }

    /** Disables simulation for all belonging machine elements (rigid bodies).
     */
    void DisableSimulation ()
    {
        for( int i = 0; i < Elements.size (); ++i ) {
            Elements.at(i)->DisableSimulation ();
        }
    }

    /** Sets the same linear velocity for all belonging machine elements (rigid bodies).
     */
    void SetLinearVelocity( const btVector3& velocity )
    {
        for( int i = 0; i < Elements.size (); ++i ) {
            Elements.at(i)->SetLinearVelocity( velocity );
        }
    }

    /** Sets the same angular velocity of all belonging machine elements (rigid bodies).
     */
    void SetAngularVelocity( const btVector3& velocity )
    {
        for( int i = 0; i < Elements.size (); ++i ) {
            Elements.at(i)->SetAngularVelocity( velocity );
        }
    }
};

#endif // _MACHINE_H_INCLUDED
