#pragma once
#ifndef _GLUT_FRAMEWORK_H_INCLUDED
#define _GLUT_FRAMEWORK_H_INCLUDED

/**
 *  @file      GLUT_Framework.h
 *  @brief     Definitions for the GLUT_Framework static class template
 *             used to wrap an instance of the DemoApplication class.
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-06-11
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "GlutStuff.h"

/////////////////////////////////////////////////////////////////////////////////////////

/** GLUT wrapper around a single instance of a GlutApplication template class.
 * GlutApplication should implement event handlers that respond to GLUT callbacks
 * (it should implement the DemoApplication interface).
 */
template<class GlutApplication>
class GLUT_Framework
{
    /////////////////////////////////////////////////////////////////////////////////////

    /** Holds an instance of the GlutApplication
     */
    static GlutApplication* Application;

    /////////////////////////////////////////////////////////////////////////////////////

    /** Called each frame to display the scene.
     */
    static void OnDisplay( void )
    {
        if ( Application ) {
            Application->displayCallback ();
        }
    }

    /** Called when a key is pressed.
     */
    static void OnKeyboard( unsigned char key, int x, int y )
    {
        if ( Application ) {
            Application->keyboardCallback( key, x, y );
        }
    }

    /** Called when a key is depressed.
     */
    static void OnKeyboardUp( unsigned char key, int x, int y )
    {
        if ( Application ) {
            Application->keyboardUpCallback( key, x, y );
        }
    }

    /** Called when a special key is pressed.
     */
    static void OnSpecialKeyboard( int key, int x, int y )
    {
        if ( Application ) {
            Application->specialKeyboard( key, x, y );
        }
    }

    /** Called when a special key is depressed.
     */
    static void OnSpecialKeyboardUp( int key, int x, int y )
    {
        if ( Application ) {
            Application->specialKeyboardUp( key, x, y );
        }
    }

    /** Called when the display window changes size.
     */
    static void OnReshape( int width, int height )
    {
        if ( Application ) {
            Application->reshape( width, height );
        }
    }

    /** Called when GLUT is idle.
     */
    static void OnIdle ()
    {
        if ( Application ) {
            Application->moveAndDisplay ();
        }
    }

    /** Called when a mouse button is pressed.
     */
    static void OnMouseButton( int button, int state, int x, int y )
    {
        if ( Application ) {
            Application->mouseFunc( button, state, x, y );
        }
    }

    /** Called when the mouse is dragged.
     */
    static void	OnMouseMotion( int x, int y )
    {
        if ( Application ) {
            Application->mouseMotionFunc( x, y );
        }
    }

    /** Called when the mouse wheel is spun.
     */
    static void OnMouseWheel( int wheel, int direction, int x, int y )
    {
        if ( Application ) {
            // Application->mouseWheelFunc( wheel, direction, x, y );
        }
    }

public:

    /////////////////////////////////////////////////////////////////////////////////////
    /** Runs GLUT main loop for the given application.
     */
    static int Main( int& argc, char* argv[] )
    {
        // Initialize GLUT environment.
        //
        glutInit( &argc, argv );

        // Create an instance of the GlutApplication.
        //
        GlutApplication application;

        // Connect GLUT event handlers to the instance.
        //
        GLUT_Framework::Application = &application;

        glutDisplayFunc       ( OnDisplay           );
        glutReshapeFunc       ( OnReshape           );
        glutIdleFunc          ( OnIdle              );
        glutKeyboardFunc      ( OnKeyboard          );
        glutKeyboardUpFunc    ( OnKeyboardUp        );
        glutSpecialFunc       ( OnSpecialKeyboard   );
        glutSpecialUpFunc     ( OnSpecialKeyboardUp );
        glutMouseFunc         ( OnMouseButton       );
        glutMotionFunc        ( OnMouseMotion       );
        glutPassiveMotionFunc ( OnMouseMotion       );
        // glutMouseWheelFunc ( OnMouseWheel        );

        // Enable vsync to avoid tearing on Apple (TODO: for Windows)
        //
        #if defined(__APPLE__) && !defined (VMDMESA)
        {
            int swap_interval = 1;
            CGLContextObj cgl_context = CGLGetCurrentContext ();
            CGLSetParameter( cgl_context, kCGLCPSwapInterval, &swap_interval );
        }
        #endif

        // Run the GlutApplication.
        //
        glutMainLoop ();

        // Disconect event handlers from the GlutApplication instance.
        //
        GLUT_Framework::Application = 0;

        return 0;
    }
};

#endif // _GLUT_FRAMEWORK_H_INCLUDED
