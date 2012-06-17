/**
 *  @file      PassiveWalkerApplication.cpp
 *  @brief     Implementation of the passive walker GLUT based application.
 *  @author    Mikica Kocic
 *  @version   0.3
 *  @date      2012-06-16
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "PassiveWalkerApplication.h"
#include "RagDoll.h"      // a rag dolls for the 'box shooter' see shootBox
#include "GLDebugFont.h"  // GLDebugDrawString

/////////////////////////////////////////////////////////////////////////////////////////

// Constructs a default passive walker test-bed (configId 1).
//
PassiveWalkerApplication::PassiveWalkerApplication ()
{
    /////////////////////////////////////////////////////////////////////////////////////

    FramesPerSecond = 60;
    RendererLoad = 0;
    RendererClock.reset ();
    CameraPositionFollowsWalker = true;
    CameraRotationFollowsWalker = false;

    /////////////////////////////////////////////////////////////////////////////////////

    const char* windowTitle  = "Lab5: Passive Walker";
    const int   windowWidth  = 800;
    const int   windowHeight = 600;

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup common options for GLUT display and newly created windows
    //
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL );

    #ifndef BT_USE_FREEGLUT
    glutInitWindowPosition(
        ( glutGet( GLUT_SCREEN_WIDTH  ) - windowWidth  ) * 1/5, 
        ( glutGet( GLUT_SCREEN_HEIGHT ) - windowHeight ) * 1/4
    );
    #endif

    glutInitWindowSize( windowWidth, windowHeight );

    /////////////////////////////////////////////////////////////////////////////////////
    // Create, initialize and pop-up main window
    //
    glutCreateWindow( windowTitle );

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup default view
    //
    myinit ();
    setTexturing( true );
    setShadows( true );
    setCameraDistance( 10 );
    m_frustumZNear = btScalar( 0.1 ),
    m_azi = 80; // sideview
    m_debugMode |= btIDebugDraw::DBG_NoHelpText;
    m_debugMode |= btIDebugDraw::DBG_DrawText;

    /////////////////////////////////////////////////////////////////////////////////////
    // Setup simulation and initialize passive walker
    //
    initPhysics ();
    TestBed.SetupPassiveWalker( /* configId */ 1 );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Destructs the BT dynamics world and all allocated objects.
//
PassiveWalkerApplication::~PassiveWalkerApplication ()
{
    RemoveAllMachines ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Initializes physics (creates BT dynamics world).
//
void PassiveWalkerApplication::initPhysics ()
{
    m_dynamicsWorld = TestBed.GetDynamicsWorld ();

    TestBed.SetDebugDrawer( &DebugDrawer );

    DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
}

// Initializes GLUT (before next rendering).
// Note: We don't use (fallback to) DemoApplication::myinit().
//
void PassiveWalkerApplication::myinit ()
{
    // ------------------------------------------------------------------ Light ---------

    GLfloat light_ambient   [] = {  0.2f,   0.2f,  0.2f,  1.0f  };
    GLfloat light_diffuse   [] = {  1.0f,   1.0f,  1.0f,  1.0f  };
    GLfloat light_specular  [] = {  1.0f,   1.0f,  1.0f,  1.0f  };
    GLfloat light_position0 [] = {  1.0f,  30.0f,  1.0f,  0.0f  };
    GLfloat light_position1 [] = { -1.0f, -30.0f, -1.0f,  0.0f  };

    glLightfv( GL_LIGHT0, GL_AMBIENT , light_ambient   );
    glLightfv( GL_LIGHT0, GL_DIFFUSE,  light_diffuse   );
    glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular  );
    glLightfv( GL_LIGHT0, GL_POSITION, light_position0 );

    glLightfv( GL_LIGHT1, GL_AMBIENT,  light_ambient   );
    glLightfv( GL_LIGHT1, GL_DIFFUSE,  light_diffuse   );
    glLightfv( GL_LIGHT1, GL_SPECULAR, light_specular  );
    glLightfv( GL_LIGHT1, GL_POSITION, light_position1 );

    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable( GL_LIGHT1 );

    // ------------------------------------------------------------------- Colors -------

    glClearColor( 0.7f, 0.7f, 0.65f, 1.0f ); // light yellowish-gray

    // ------------------------------------------------------------------- Flags --------

    glShadeModel( GL_SMOOTH );
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LESS );

    glEnable( GL_CULL_FACE );
    glCullFace( GL_BACK );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Steps the simulation and renders the dynamics world.
//
void PassiveWalkerApplication::clientMoveAndDisplay ()
{
    if ( m_dynamicsWorld )
    {
        // Step simulation for duration dependent on renderer load
        //
        double duration = ( RendererLoad <= 1.0 ? 1.0 : RendererLoad ) / FramesPerSecond;

        TestBed.StepSimulation( btScalar( duration ), isIdle () );

        // Restart the on-going Monte Carlo simulation
        //
        if ( TestBed.IsMonteCarloTestSuite () ) {
            TestBed.SetupPassiveWalker( 0 );
            PurgeShapeCache ();
            m_azi = 80; // sideview
        }
    }

    // Render simulated objects
    //
    renderme ();
    AdaptiveFps ();
}

// GLUT event handler that renders the dynamics world.
//
void PassiveWalkerApplication::displayCallback ()
{
    renderme ();
    AdaptiveFps ();
}

// Renders the dynamics world.
//
void PassiveWalkerApplication::renderme ()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glRasterPos3f( 0, 0, 0 );

    DemoApplication::renderme ();

    TestBed.DebugDrawWorld ();
    DrawDebugText ();

    glFlush ();
    swapBuffers ();
}

// Swaps GL buffers.
//
void PassiveWalkerApplication::swapBuffers ()
{
	glutSwapBuffers ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Calculates render load and pauses, if needed, adapting to the given nominal FPS.
//
void PassiveWalkerApplication::AdaptiveFps ()
{
    if ( TestBed.IsMonteCarloTestSuite () ) {
        RendererClock.reset ();
        return;
    }

    const double framePeriod = 1.0 / FramesPerSecond;
    const double MinFPS = 25;
    const double MaxFPS = 60;

    // Get elapsed time
    //
    double elapsed = RendererClock.getTimeMicroseconds() * 1e-6;

    // Sleep, if we have time, and restart renderer timer
    //
    Pause( framePeriod - elapsed - 0.2e-3 );
    RendererClock.reset ();

    // printf( "%.6f\n", elapsed );

    // Calculate renderer load using exponential average algorithm.
    //
    RendererLoad = 0.99 * RendererLoad + 0.01 * elapsed / framePeriod;

    // In case of heavily used renderer, decrease FPS.
    //
    if ( RendererLoad > 1 && FramesPerSecond > MinFPS ) {
        FramesPerSecond /= RendererLoad;
    }

    // If renderer usage drops bellow 98 %, reset FPS to max value.
    //
    if ( RendererLoad < 1 && FramesPerSecond < MaxFPS ) {
        FramesPerSecond /= RendererLoad;
    }

    // Keep nominal FPS between MinFPS and MaxFPS
    //
    if ( FramesPerSecond < MinFPS ) {
        FramesPerSecond = MinFPS;
    }
    else if ( FramesPerSecond > MaxFPS ) {
        FramesPerSecond = MaxFPS;
    }
}

// Updates the GL camera view.
//
void PassiveWalkerApplication::updateCamera ()
{
    if ( ! isIdle () ) {
        ChangeCameraView ();
    }

    DemoApplication::updateCamera ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Called when a key is depressed.
//
void PassiveWalkerApplication::keyboardCallback( unsigned char key, int x, int y )
{
    bool wireFrame = ( DebugDrawer.getDebugMode() & btIDebugDraw::DBG_DrawWireframe ) != 0;

    switch( key )
    {
        case 'e':
        {
            RagDoll* rd = new RagDoll( m_dynamicsWorld, Transform( 0, 3, 0 ), 1, false, 2 );
            rd->EnableDeactivation ();
            Machines.push_back( rd );
            glutPostRedisplay ();
            return;
            }

        case '\r':
        case 's':
            m_idle = true;
            DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
            clientMoveAndDisplay ();
            return;

        case 'S':
            m_idle = true;
            DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
            clientMoveAndDisplay ();
            ChangeCameraView ();
            return;

        case 'i':
            DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
            toggleIdle ();
            return;

        case '/':
            DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
            TestBed.ToggleTimeScale ();
            return;

        case ' ':
            m_idle = false;
            RemoveAllMachines ();
            TestBed.RestartPassiveWalker ();
            PurgeShapeCache ();
            m_azi = TestBed.GetTestSuiteId () == 8 ? btScalar( 120 ) : btScalar( 80 );
            ChangeCameraView ();
            glutPostRedisplay ();
            return;
    }

    PassiveWalker* walker = TestBed.GetWalker ();

    if ( walker && walker->IsAutonomous () )
    {
        switch( key )
        {
            case '1':
                walker->Parameters.HipAngleLimit -= btRadians( btScalar( 1.0 ) );
                walker->MotorParametersVerified = false;
                return;

            case '2':
                walker->Parameters.HipAngleLimit += btRadians( btScalar( 1.0 ) );
                walker->MotorParametersVerified = false;
                return;

            case '3':
                walker->Parameters.LegAngularVelocity -= btRadians( btScalar( 1.0 ) );
                walker->MotorParametersVerified = false;
                return;

            case '4':
                walker->Parameters.LegAngularVelocity += btRadians( btScalar( 1.0 ) );
                walker->MotorParametersVerified = false;
                return;

            case '5':
                walker->Parameters.DirectionDiff -= btRadians( btScalar( 0.1 ) );
                walker->MotorParametersVerified = false;
                return;

            case '6':
                walker->Parameters.DirectionDiff += btRadians( btScalar( 0.1 ) );
                walker->MotorParametersVerified = false;
                return;
        }
    }

    DemoApplication::keyboardCallback( key, x, y );

    if ( wireFrame ) {
        DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Called when a special key is pressed.
//
void PassiveWalkerApplication::specialKeyboard( int key, int x, int y )
{
    (void) x, y;

    // const int state = glutGetModifiers ();
    // state & BT_ACTIVE_SHIFT 

    DebugDrawer.setDebugMode( btIDebugDraw::DBG_DrawWireframe );

    switch( key ) 
    {
        case GLUT_KEY_F1:   SetupSimulation( 1 );  break;
        case GLUT_KEY_F2:   SetupSimulation( 2 );  break;
        case GLUT_KEY_F3:   SetupSimulation( 3 );  break;
        case GLUT_KEY_F4:   SetupSimulation( 4 );  break;
        case GLUT_KEY_F5:   SetupSimulation( 5 );  break;
        case GLUT_KEY_F6:   SetupSimulation( 6 );  break;
        case GLUT_KEY_F7:   SetupSimulation( 7 );  break;
        case GLUT_KEY_F8:   SetupSimulation( 8 );  break;
        case GLUT_KEY_F11:  TestBed.Dump ();       break;
        case GLUT_KEY_F12:  SetupSimulation( 0 );  break;

        case GLUT_KEY_LEFT:      stepLeft   ();    break;
        case GLUT_KEY_RIGHT:     stepRight  ();    break;
        case GLUT_KEY_UP:        stepFront  ();    break;
        case GLUT_KEY_DOWN:      stepBack   ();    break;
        case GLUT_KEY_PAGE_UP:   zoomIn     ();    break;
        case GLUT_KEY_PAGE_DOWN: zoomOut    ();    break;
    }

	glutPostRedisplay ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Called when a special key is pressed.
//
void PassiveWalkerApplication::updateModifierKeys ()
{
    m_modifierKeys = 0;

    if ( ( glutGetModifiers () & GLUT_ACTIVE_ALT ) != 0 ) {
        m_modifierKeys |= BT_ACTIVE_ALT;
    }

    if ( ( glutGetModifiers () & GLUT_ACTIVE_CTRL ) != 0 ) {
        m_modifierKeys |= BT_ACTIVE_CTRL;
    }

    if ( ( glutGetModifiers() & GLUT_ACTIVE_SHIFT ) != 0 ) {
        m_modifierKeys |= BT_ACTIVE_SHIFT;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Shoots a cube or a rag doll.
//
void PassiveWalkerApplication::shootBox( const btVector3& destination )
{
if ( ! m_dynamicsWorld ) {
        return;
    }

    DebugDrawer.setDebugMode( m_debugMode );

    // If heavy loaded renderer, shoot cubes 
    //
    if ( RendererLoad > 0.6 ) {
        DemoApplication::shootBox( destination );
        return;
    }

    // Otherwise, shoot rag dolls
    //
    btVector3 camPos = getCameraPosition ();
    btVector3 linVel = destination - camPos;
    btScalar  speed  = linVel.length ();

    if ( speed > 0 )
    {
    linVel *= m_ShootBoxInitialSpeed/speed/3;
        camPos += linVel * 0.2f;

        RagDoll* rd = new RagDoll( m_dynamicsWorld, Transform(camPos), 0.1f, false, 1 );
        rd->EnableDeactivation ();
        rd->SetLinearVelocity( linVel );
        Machines.push_back( rd );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Removes all machines allocated during the simulation.
//
void PassiveWalkerApplication::RemoveAllMachines ()
{
    for ( int i = Machines.size () - 1; i >= 0; --i )
    {
        delete Machines.at(i);
    }

    Machines.clear ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Removes the shape cache allocated by DemoApplication.
//
void PassiveWalkerApplication::PurgeShapeCache ()
{
    if ( m_shapeDrawer )
    {
        bool textureEnabled = m_shapeDrawer->hasTextureEnabled ();
        delete m_shapeDrawer;
        m_shapeDrawer = new GLShapeDrawer ();
        m_shapeDrawer->enableTexture( textureEnabled );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Change camera view according to walker's position.
//
void PassiveWalkerApplication::ChangeCameraView ()
{
    if ( CameraPositionFollowsWalker )
    {
        const btTransform& T( TestBed.GetWalker()->HipAxis->getWorldTransform () );

        m_cameraTargetPosition = T.getOrigin ();

        const btVector3& offset =
            TestBed.GetGroundPlane()->getCenterOfMassPosition ();

        m_cameraTargetPosition.setY( 
            offset.y() + ( m_cameraTargetPosition.y() - offset.y() ) * 0.3f
        );

        if ( ! TestBed.GetWalker()->IsAutonomous () ) {
            m_azi += 0.01f;
        }

        if ( CameraRotationFollowsWalker )
        {
            btScalar myAzimuth = 0;

            // Cut down version for getting yaw from basis from the Ogre source code
            // OgreMatrix3.cpp:ToEulerYXZ ()
            //
            const btMatrix3x3& m = T.getBasis ();
            const btScalar rfPAngle = btAsin( -m[1][2] );

            btScalar azi = rfPAngle >  SIMD_HALF_PI ?  btAtan2( -m[0][1], m[0][0] )
                         : rfPAngle > -SIMD_HALF_PI ?  btAtan2(  m[0][2], m[2][2] )
                                                    : -btAtan2( -m[0][1], m[0][0] );

            m_azi = myAzimuth + btDegrees( azi );
        }

        if ( m_azi < 0 ) {
            m_azi += 360;
        }
        else if ( m_azi >= 360 ) {
            m_azi -= 360;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

// Configures the simulation test-bed (walker and ground plane) according to the
// given configuration id.
//
void PassiveWalkerApplication::SetupSimulation( int testSuiteId )
{
    if ( testSuiteId == 0 ) {
        TestBed.SetWalkDurationGoal( 0.0 ); 
    }

    RemoveAllMachines ();
    TestBed.SetupPassiveWalker( testSuiteId );
    PurgeShapeCache ();

    // In case of active walker:
    // CameraRotationFollowsWalker = testSuiteId == 8;
    m_azi = testSuiteId == 8 ? btScalar( 120 ) : btScalar( 80 );

    ChangeCameraView ();
    glutPostRedisplay ();
}

/////////////////////////////////////////////////////////////////////////////////////////

// Draws a debug info (local time, test suite id, configuration etc).
//
void PassiveWalkerApplication::DrawDebugText ()
{
    if(    ( m_debugMode & btIDebugDraw::DBG_DrawText   ) == 0
        || ( m_debugMode & btIDebugDraw::DBG_NoHelpText ) == 0 )
    {
        return;
    }

    int xStart = 10;
    int yStart = 20;

    setOrthographicProjection ();
    glDisable( GL_LIGHTING );
    glColor3f( 1, 1, 1 );

    #ifdef _MSC_VER
        #pragma warning(disable:4996) // (v)sprintf warning
    #endif

    char buf[1024];

    // Renderer usage and FPS

    if ( ! TestBed.IsMonteCarloTestSuite () )
    {
        double actualFramesPerSecond = 
            RendererLoad <= 1.0 ? FramesPerSecond : FramesPerSecond / RendererLoad;

        int count = sprintf( buf, "Load %3.lf %%", RendererLoad * 100 );
        GLDebugDrawString( m_glutScreenWidth - count * 10 - 15, yStart, buf );

        count = sprintf( buf, "%4.lf fps", actualFramesPerSecond );
        GLDebugDrawString( m_glutScreenWidth - count * 10 - 15, yStart + 20, buf );
    }

    // Report test suite id, local time and time scale

    if ( TestBed.IsMonteCarloTestSuite () ) {
        sprintf( buf, "Running Monte Carlo: goal = %lg, t = %.3lf", 
            TestBed.GetWalkDurationGoal (), TestBed.GetLocalTime () );
    }
    else if ( TestBed.GetTimeScale () != 1 ) {
        sprintf( buf, "t = %.3lf s (%.2lfx), Test Suite: %d", 
            TestBed.GetLocalTime (), TestBed.GetTimeScale (), TestBed.GetTestSuiteId () );
    }
    else {
        sprintf( buf, "t = %.3lf s, Test Suite: %d", 
            TestBed.GetLocalTime (), TestBed.GetTestSuiteId () );
    }

    GLDebugDrawString( xStart, yStart, buf );
    yStart += 20;

    // Now, the walker state variables and parameters
    //
    PassiveWalker* walker = TestBed.GetWalker ();

    if ( walker )
    {
        // The ground plane inclination

        sprintf( buf, "Inclination/deg: %.2lf", 
            btDegrees( walker->Parameters.InclinationAngle )
        );
        GLDebugDrawString( xStart, yStart, buf );
        yStart += 20;

        // Display the motor parameters
        //
        if ( ! walker->IsAutonomous () )
        {
            sprintf( buf, "Motor: Off (passive mode)\n" );
        }
        else
        {
            sprintf( buf, "Motor: Hip %.lf, Leg %.lf, Dir %.1lf %s", 
                double( btDegrees( walker->Parameters.HipAngleLimit ) ),
                double( btDegrees( walker->Parameters.LegAngularVelocity ) ),
                double( btDegrees( walker->Parameters.DirectionDiff ) ),
                walker->MotorParametersVerified ? "(Active)" : ""
                );
        }
        GLDebugDrawString( xStart, yStart, buf );
        yStart += 20;

        // Display hip and knees angle sensor
        //
        sprintf( buf, "Angle/deg: Hip %5.1lf, IL %5.1lf, OR %5.1lf ", 
            double( btDegrees( walker->HipAngle ) ),
            double( btDegrees( walker->Inner.Left.Knee->getAngle(0) ) ),
            double( btDegrees( walker->Outer.Right.Knee->getAngle(0) ) )
            );
        GLDebugDrawString( xStart, yStart, buf );
        yStart += 20;

        // Display feet ground contact sensors
        //
        sprintf( buf, "Contact Sensors: %s %s %s %s %s",
           walker->Inner.Left .FootGroundContact ? "IL" : "--",
           walker->Inner.Right.FootGroundContact ? "IR" : "--",
           walker->Outer.Left .FootGroundContact ? "OL" : "--",
           walker->Outer.Right.FootGroundContact ? "OR" : "--",
           walker->HipGroundContact ? "Hip" : ""
        );
        GLDebugDrawString( xStart, yStart, buf );
        yStart += 20;
    }

    resetPerspectiveProjection ();
    glEnable( GL_LIGHTING );
}

/////////////////////////////////////////////////////////////////////////////////////////
// Implementation of platform dependent methods

#ifdef _WIN32
    #include <Windows.h>
#else
    #include <unistd.h>
#endif

// Pauses execution (relaxes CPU) for the given duration in seconds.
//
void PassiveWalkerApplication::Pause( double duration )
{
    if ( duration > 0 )
    {
        #ifdef _WIN32
            Sleep( DWORD( duration * 1e3 + 0.5 ) );
        #else
            usleep( (unsigned long)( duration * 1e6 + 0.5 ) );
        #endif
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
