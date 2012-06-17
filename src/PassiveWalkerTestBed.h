#pragma once
#ifndef _PASSIVE_WALKER_TEST_BED_H_INCLUDED
#define _PASSIVE_WALKER_TEST_BED_H_INCLUDED

/**
 *  @file      PassiveWalkerTestBed.h
 *  @brief     Definitions for the PassiveWalkerTestBed class that encapsulates
 *             a test bed used for testing passive walkers.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Machine.h"
#include "PassiveWalker.h"

/////////////////////////////////////////////////////////////////////////////////////////

class btDefaultCollisionConfiguration;

/** Encapsulates a test bed for testing simulations for the PassiveWalker class.
 */
class PassiveWalkerTestBed : public Machine
{
    /** Holds the Bullet collision detection stack allocator, pool memory allocators etc.
     */
    btDefaultCollisionConfiguration* CollisionConfiguration;

    /** Holds the local time in seconds.
     */
    volatile double LocalTime;

    /** Holds the time scale difference between the simulated and the real-time.
     */
    btScalar TimeScale;

    /** Indicates whether to use the time scale when rendering the dynamics worl.
     */
    bool EnableTimeScale;

    /** Holds the current instance of the passive walker.
     */
    PassiveWalker* Walker;

    /** Holds the ground plane object.
     */
    Machine::Element GroundPlane;

    /** Holds the current test-suite id.
     */
    int TestSuiteId;

    /** Holds the last test-suite id (usedl when running Monte Carlo simulations).
     */
    int LastTestSuiteId;

    /** Indicates whether the Monte Carlo simulation runs for the given test-suite id.
     */
    volatile bool MonteCarlo;

    /** Holds the 'walk duration' goal to be achived by the Monte Carlo.
     */
    double WalkDurationGoal;

public:

    /** Constructs a default dynamics world, but without any passive walker.
     */
    PassiveWalkerTestBed ();

    /** Destructs the dynamics world with all its objects.
     */
    ~PassiveWalkerTestBed ();

    /** Gets the current instance the passive walker.
     */
    PassiveWalker* GetWalker () const
    {
        return Walker;
    }

    /** Gets the reference to the ground plane.
     */
    const Machine::Element& GetGroundPlane () const
    {
        return GroundPlane;
    }

    /** Gets the local time.
     */
    double GetLocalTime () const
    {
        return LocalTime;
    }

    /** Gets the time scale.
     */
    double GetTimeScale () const
    {
        return EnableTimeScale ? double( TimeScale ) : 1.0;
    }

    /** Sets the current 'walk duration' goal for the Monte Carlo.
     */
    void SetWalkDurationGoal( double duration )
    {
        WalkDurationGoal = duration;
    }

    /** Gets the current 'walk duration' goal for the Monte Carlo.
     */
    double GetWalkDurationGoal () const
    {
        return WalkDurationGoal;
    }

    /** Gets the current test-suite id.
     */
    int GetTestSuiteId () const
    {
        return TestSuiteId;
    }

    /** Returns true if the Monte Carlo test-suite is running.
     */
    bool IsMonteCarloTestSuite () const
    {
        return TestSuiteId == 0;
    }

    /** Displays the state variables of the passive walker.
     */
    void Dump () const
    {
        if ( Walker ) {
            Walker->Dump( LocalTime );
        }
    }

    /** Toggles using the time-scale during the dynamics world rendering.
     */
    void ToggleTimeScale ()
    {
        EnableTimeScale = ! EnableTimeScale;
    }

    /** Executes simulation for the given duration in seconds.
     */
    void StepSimulation( btScalar duration, bool singleStep );

    /** Creates the passive walker and ground plane according to the given test-suite id.
     */
    void SetupPassiveWalker( int testSuiteId );

    /** Restarts the current passive walker simulation (using its initial conditions).
     */
    void RestartPassiveWalker ();

private:

    /** Event handler called on before every simulation substep tick.
     * Updates walker's finite-state machine and angular motors.
     */
    void OnPreTick( btScalar timeStep );

    /** Event handler called on after every simulation substep tick.
     * Updates local time.
     */
    void OnTick( btScalar timeStep );

    /** Called before an internal tick (before simulation substep) happens.
     */
    static void OnPreTickCallback( btDynamicsWorld* world, btScalar timeStep )
    {
        ( (PassiveWalkerTestBed*)world->getWorldUserInfo () )->OnPreTick( timeStep );
    }

    /** Called when an internal tick (after simulation substep) happens.
     */
    static void OnTickCallback( btDynamicsWorld* world, btScalar timeStep )
    {
        ( (PassiveWalkerTestBed*)world->getWorldUserInfo () )->OnTick( timeStep );
    }
};

#endif // _PASSIVE_WALKER_TEST_BED_H_INCLUDED
