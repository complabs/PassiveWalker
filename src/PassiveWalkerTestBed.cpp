/**
 *  @file      PassiveWalkerTestBed.cpp
 *  @brief     Implementation of the PassiveWalkerTestBed class that encapsulates
 *             a test bed used for testing passive walkers.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "PassiveWalkerTestBed.h"
#include "Constants.h"
#include "Utilities.h"

#include "btBulletDynamicsCommon.h"

#include <cstdio>

/////////////////////////////////////////////////////////////////////////////////////////

// Constructs a default dynamics world, but without any passive walker.
//
PassiveWalkerTestBed::PassiveWalkerTestBed ()
    : LocalTime       ( 0 )
    , TimeScale       ( 1 )
    , EnableTimeScale ( false )
    , Walker          ( 0 )
    , TestSuiteId     ( 1 )
    , LastTestSuiteId ( 1 )
    , MonteCarlo      ( false )
    , WalkDurationGoal( 0 )
{
    btScalar worldSize = btScalar( 1e4 );
    btVector3 worldBox( worldSize, worldSize, worldSize );

    CollisionConfiguration = new btDefaultCollisionConfiguration;
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
    btBroadphaseInterface* broadphase = new btAxisSweep3( -worldBox, worldBox );
    btDispatcher* dispatcher = new btCollisionDispatcher( CollisionConfiguration );

    World = new btDiscreteDynamicsWorld(
        dispatcher, broadphase, solver, CollisionConfiguration
    );

    // Setup default standard gravity
    //
    World->setGravity( Const::g_n );

    // Setup dynamics world pre & post tick event handlers
    //
    World->setInternalTickCallback( OnPreTickCallback, this, /*isPreTick*/ true  );
    World->setInternalTickCallback( OnTickCallback,    this, /*isPreTick*/ false );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Destructs the dynamics world with all its objects.
//
PassiveWalkerTestBed::~PassiveWalkerTestBed ()
{
    if ( Walker ) {
        // World->removeAction( Walker );
        delete Walker;
    }

    GroundPlane.Disconnect ();

    // Delete allocated objects in reverse order (see the constructor).
    //
    if ( World )
    {
        btConstraintSolver*    solver     = World->getConstraintSolver ();
        btBroadphaseInterface* broadphase = World->getBroadphase ();
        btDispatcher*          dispatcher = World->getDispatcher ();

        delete World;
        delete dispatcher;
        delete broadphase;
        delete solver;
        delete CollisionConfiguration;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Executes simulation for the given duration in seconds.
//
void PassiveWalkerTestBed::StepSimulation( btScalar duration, bool singleStep )
{
    if ( ! World ) {
        return;
    }

    World->stepSimulation( 
        /* timeStep    */ EnableTimeScale ? duration * TimeScale : duration,
        /* maxSubSteps */ singleStep ? 1 : 1000
    );

    // Continue with Monte Carlo
    //
    if ( LastTestSuiteId == 0 && Walker && Walker->HipGroundContact ) {
        SetupPassiveWalker( 0 );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Event handler called on before every simulation substep tick.
// Updates walker's finite-state machine and angular motors.
//
void PassiveWalkerTestBed::OnPreTick( btScalar timeStep )
{
    if ( Walker )
    {
        // Update the walker's state machine
        //
        Walker->StateMachine( LocalTime, timeStep );

        // If the walker has fallen _off_ the plane (being disabled) or 
        // if it has fallen _on_ the ground, turn-off Monte Carlo mode.
        //
        if ( MonteCarlo || LastTestSuiteId == 0 )
        {
            if ( Walker->IsDisabledSimulation () || Walker->HipGroundContact ) {
                MonteCarlo = false;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Called on after every simulation substep tick.
//
void PassiveWalkerTestBed::OnTick( btScalar timeStep )
{
    LocalTime += timeStep;
}

/////////////////////////////////////////////////////////////////////////////////////////

// Restarts the current passive walker simulation (using its initial conditions).
//
void PassiveWalkerTestBed::RestartPassiveWalker ()
{
    printf( "Restarting the current walker; t = %lg s\n", LocalTime );

    // Reset local time only (without resetting current time scale)
    //
    LocalTime = 0;

    // Initialize parameters for a new passive walker
    //
    PassiveWalker* oldWalker = Walker;
    Walker = new PassiveWalker( World );

    // Get parameters from the old walker
    //
    if ( oldWalker )
    {
        Walker->Parameters = oldWalker->Parameters;
        Walker->Inits = oldWalker->Inits;
        if ( oldWalker->IsAutonomous () ) {
            Walker->State = PassiveWalker::RECOVERING;
        }
        delete oldWalker;
    }

    // (Re)create the ground plane
    //
    const PassiveWalker::ConstructionInfo& p = Walker->Parameters;

    if ( p.PlaneThickness <= 0 ) // as a static plane
    {
        GroundPlane.PlaneShape( this, 
            btVector3( 0, p.PlaneOffsetY, 0 ),
            btVector3( p.InclinationAngle, 0, 0 )
        );
    }
    else // as a big thin box
    {
        GroundPlane.BoxShape( this, 0, btTransform::getIdentity (),
            btVector3( 0, p.PlaneOffsetY - p.PlaneThickness, 0 ),
            btVector3( p.InclinationAngle, 0, 0 ),
            btVector3( p.PlaneSize/2, p.PlaneThickness, p.PlaneSize/2 ), 1
        );
    }

    // Initialize the current walker and establish ground plane sensors
    //
    Walker->Create ();
    Walker->SetupGroundPlaneSensors( GroundPlane );
    // World->addAction( Walker );

    Walker->Dump( LocalTime );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Creates the passive walker and ground plane according to the given test-suite id.
//
void PassiveWalkerTestBed::SetupPassiveWalker( int testSuiteId )
{
    // Remember the current configuration and setup a new one
    //
    if ( ! IsMonteCarloTestSuite () ) {
        LastTestSuiteId = TestSuiteId;
    }
    TestSuiteId = testSuiteId;

    if ( ! IsMonteCarloTestSuite () ) {
        printf( "\nStarting a passive walker from test suite %d...\n", testSuiteId );
    }

    /////////////////////////////////////////////////////////////////////////////////////

    // Reset the local time and the time scale
    //
    LocalTime = 0;
    TimeScale = 1;

    // Remove the old passive walker and create a new passive walker
    //
    if ( Walker ) {
        // World->removeAction( Walker );
        delete Walker;
    }

    Walker = new PassiveWalker( World );

    /////////////////////////////////////////////////////////////////////////////////////

    Walker->Parameters.Position.setValue( 0, Walker->Parameters.PlaneOffsetY, 0 );

    // Get the configuration for a new walker according our TestSuiteId
    //
    if ( IsMonteCarloTestSuite () ) // Monte Carlo simulation based on the last suite id
    {
        Walker->SetupInitialConditions( LastTestSuiteId );
        Walker->RandomizeInitialConditions ();

        if ( Walker->Inits.WalkDurationGoal > WalkDurationGoal ) {
            WalkDurationGoal = Walker->Inits.WalkDurationGoal;
        }
    }
    else // normal simulation
    {
        Walker->SetupInitialConditions( TestSuiteId );
    }

    // Create the ground plane
    //
    const PassiveWalker::ConstructionInfo& p = Walker->Parameters;

    if ( p.PlaneThickness <= 0 ) // as a static plane
    {
        GroundPlane.PlaneShape( this, 
            btVector3( 0, p.PlaneOffsetY, 0 ),
            btVector3( p.InclinationAngle, 0, 0 )
        );
    }
    else // as a big thin box
    {
        GroundPlane.BoxShape( this, 0, btTransform::getIdentity (),
            btVector3( 0, p.PlaneOffsetY - p.PlaneThickness, 0 ),
            btVector3( p.InclinationAngle, 0, 0 ),
            btVector3( p.PlaneSize/2, p.PlaneThickness, p.PlaneSize/2 ), 1
        );
    }

    // Initialize the current walker and establish ground plane sensors
    //
    Walker->Create ();
    Walker->SetupGroundPlaneSensors( GroundPlane );
    // World->addAction( Walker );

    // Adjust time-scale to correspond to walker length-scale
    //
    TimeScale = btSqrt( Walker->Parameters.LengthUnits / 1e-3f );

    if ( ! IsMonteCarloTestSuite () )
    {
        Walker->Dump( LocalTime );
    }
    else // Monte Carlo simulation
    {
        static int monteCarloCounter = 0; // simulation counter
        ++monteCarloCounter;

        printf( "\nRunning Monte Carlo %d... ", monteCarloCounter );

        const double simulationTimeLimit = 50.0; // max simulation time in seconds

        // Do the simulation until the walker falls or time limit has been reached.
        // Note: OnTick() updates both MonteCarlo and LocalTime (declared as volatile).
        //
        MonteCarlo = true; 
        while( MonteCarlo && LocalTime < simulationTimeLimit )
        {
            World->stepSimulation( btScalar( 0.1 ) );
        }

        // In case of the time limit reached, restart the Monte Carlo simulation.
        //
        if ( LocalTime >= simulationTimeLimit )
        {
            printf( "timeout", LocalTime );
            MonteCarlo = false;
            return;
        }

        printf( "walk duration = %.2lf s", LocalTime );

        // Otherwise, if it was the longest walk til now, report the inital parameters.
        //
        if ( Walker->IsDisabledSimulation () || LocalTime > WalkDurationGoal )
        {
            WalkDurationGoal = LocalTime;

            // Report the current initial parameters
            //
            printf( "\n" );
            Walker->Dump( LocalTime );
            printf( "---------------------------------------------------\n" );
            printf( "Inclination: %.16lf\n", Walker->Parameters.InclinationAngle );
            Walker->Inits.Dump ();
            printf( "---------------------------------------------------\n" );
            fflush( stdout );

            Walker->SaveInitialConditions( LastTestSuiteId, LocalTime );

            // Quit the Monte Carlo mode
            //
            TestSuiteId = LastTestSuiteId;

            // If the walker hasn't fallen off the plane, remember that we were in 
            // the MC mode; setting LastTestSuiteId to 0 will force the continuation 
            // of the Monte Carlo mode.
            //
            if ( ! Walker->IsDisabledSimulation () ) {
                LastTestSuiteId = 0;
            }

            // Show the last walk (restart the walker with the current parameters)
            //
            RestartPassiveWalker ();
        }
    }
}
