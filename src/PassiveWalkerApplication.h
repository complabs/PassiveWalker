#pragma once
#ifndef _PASSIVE_WALKER_APPLICATION_H_INCLUDED
#define _PASSIVE_WALKER_APPLICATION_H_INCLUDED

/**
 *  @file      PassiveWalkerApplication.h
 *  @brief     Definitions for the passive walker GLUT based application.
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-06-11
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "PassiveWalkerTestBed.h"

#include "DemoApplication.h"
#include "GLDebugDrawer.h"

/////////////////////////////////////////////////////////////////////////////////////////

class PassiveWalkerApplication : public DemoApplication
{
    /** Holds the nominal ('wished') frames per second.
     */
    double FramesPerSecond;

    /** Holds the BT clock used to measure a renderer load.
     */
    btClock RendererClock;

    /** Holds the last measured renderer load; 0 means idle, > 1 heavy loaded.
     */
    double RendererLoad;

    /** Indicates whether the camera follows the walker's position.
     */
    bool CameraPositionFollowsWalker;

    /** Indicates whether the camera follows the walker's orientation.
     */
    bool CameraRotationFollowsWalker;

    /** Holds the test-bed used for testing simulations.
     */
    PassiveWalkerTestBed TestBed;

    /** Holds an instance of the debug drawer for the GLUT.
     */
    GLDebugDrawer DebugDrawer;

    /** Holds the collection of allocated machines (like walkers or shooted rag dolls).
     */
    btAlignedObjectArray<Machine*> Machines;

private:

    /** Calculates render load and pauses, if needed, adapting to the given nominal FPS.
     */
    void AdaptiveFps ();

    /** Draws a debug info (local time, test suite id, configuration etc).
     */
    void DrawDebugText ();

    /** Removes all machines allocated during the simulation.
     */
    void RemoveAllMachines ();

    /** Removes the shape cache allocated by DemoApplication.
     * NOTE: DemoApplication has a memory leak. It does not remove allocated shape
     * from the shape cache when the user removes the rigid body from the dynamics world.
     */
    void PurgeShapeCache ();

    /** Configures the simulation test-bed (walker and ground plane) according to the
     * given configuration id.
     */
    void SetupSimulation( int testSuiteId );

    /** Updates the camera view according to walker's position.
     */
    void ChangeCameraView ();

public:

    /** Constructs a default passive walker test-bed (configId 1).
     */
    PassiveWalkerApplication ();

    /** Destructs the BT dynamics world and all allocated objects.
     */
    ~PassiveWalkerApplication ();

    /** Initializes GLUT (before next rendering).
     */
    virtual void myinit ();

    /** Initializes physics (creates BT dynamics world).
     */
    virtual void initPhysics ();

    /** Steps the simulation and renders the dynamics world.
     */
    virtual void clientMoveAndDisplay ();

    /* GLUT event handler that renders the dynamics world.
     */
    virtual void displayCallback ();

    /** Renders the dynamics world.
     */
    virtual void renderme ();

    /** Swaps GL buffers.
     */
	virtual void swapBuffers ();

    /** Updates the GL camera view.
     */
    virtual void updateCamera ();

    /** Called when a key is depressed.
     */
    virtual void keyboardCallback( unsigned char key, int x, int y );

    /** Called when a special key is pressed.
     */
    virtual void specialKeyboard( int key, int x, int y );

    /** Updates the key modifiers (the state of shift, alt etc).
     */
	virtual	void updateModifierKeys ();

    /** Shoots a cube or a rag doll.
     */
    virtual void shootBox( const btVector3& destination );

    /** Pauses execution (relaxes CPU) for the given duration in seconds.
     */
    static void Pause( double duration );
};

#endif // _PASSIVE_WALKER_APPLICATION_H_INCLUDED
