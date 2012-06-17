#pragma once
#ifndef _GLUT_STUFF_H_INCLUDED
#define _GLUT_STUFF_H_INCLUDED

/*
    Bullet Continuous Collision Detection and Physics Library
    Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

    This software is provided 'as-is', without any express or implied warranty.
    In no event will the authors be held liable for any damages arising from the use 
    of this software. Permission is granted to anyone to use this software for any purpose, 
    including commercial applications, and to alter it and redistribute it freely, 
    subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not claim that 
       you wrote the original software. If you use this software in a product, an 
       acknowledgment in the product documentation would be appreciated but is not 
       required.
    2. Altered source versions must be plainly marked as such, and must not be 
       misrepresented as being the original software.
    3. This notice may not be removed or altered from any source distribution.

    -------------------------------
    Modified 2012-06-12 by MBKocic:

    - Removed GlutStuff.cpp and integrated with the GLUT_Framework class from Lab4.
    - Modified to suite FreeGLUT
    - Removed references to _WINDOWS define
*/

/////////////////////////////////////////////////////////////////////////////////////////

#if defined( BT_USE_FREEGLUT )

    /** Disable GLUT ataxit workaround on windows.
     *
     *  Win32 has an annoying issue where there are multiple C run-time
     *  libraries (CRTs). If the executable is linked with a different CRT
     *  from the GLUT DLL, the GLUT DLL will not share the same CRT static
     *  data seen by the executable.  In particular, atexit callbacks registered
     *  in the executable will not be called if GLUT calls its (different)
     *  exit routine).  GLUT is typically built with the
     *  "/MD" option (the CRT with multithreading DLL support), but the Visual
     *  C++ linker default is "/ML" (the single threaded CRT).
     *
     *  One workaround to this issue is requiring users to always link with
     *  the same CRT as GLUT is compiled with.  That requires users supply a
     *  non-standard option.  GLUT 3.7 has its own built-in workaround where
     *  the executable's "exit" function pointer is covertly passed to GLUT.
     *  GLUT then calls the executable's exit function pointer to ensure that
     *  any "atexit" calls registered by the application are called if GLUT
     *  needs to exit.
     *
     *  To avoid the atexit workaround, define GLUT_DISABLE_ATEXIT_HACK.
     */
    #define GLUT_DISABLE_ATEXIT_HACK

    #include "GL/freeglut.h"

#elif defined( __APPLE__ ) && ! defined( VMDMESA ) // think different

    #include <OpenGL/OpenGL.h>
    #include <OpenGL/gl.h>
    #include <OpenGL/glu.h>
    #include <GLUT/glut.h>

#else // Common GLUT

    #include <GL/gl.h>
    #include <GL/glut.h>

#endif

/////////////////////////////////////////////////////////////////////////////////////////

#define BT_KEY_K            'k'
#define BT_KEY_LEFT          GLUT_KEY_LEFT
#define BT_KEY_RIGHT         GLUT_KEY_RIGHT
#define BT_KEY_UP            GLUT_KEY_UP
#define BT_KEY_DOWN          GLUT_KEY_DOWN
#define BT_KEY_F1            GLUT_KEY_F1
#define BT_KEY_F2            GLUT_KEY_F2
#define BT_KEY_F3            GLUT_KEY_F3
#define BT_KEY_F4            GLUT_KEY_F4
#define BT_KEY_F5            GLUT_KEY_F5
#define BT_KEY_PAGEUP        GLUT_KEY_PAGE_UP
#define BT_KEY_PAGEDOWN      GLUT_KEY_PAGE_DOWN
#define BT_KEY_END           GLUT_KEY_END
#define BT_KEY_HOME          GLUT_KEY_HOME
#define BT_ACTIVE_ALT        GLUT_ACTIVE_ALT
#define BT_ACTIVE_CTRL       GLUT_ACTIVE_ALT
#define BT_ACTIVE_SHIFT      GLUT_ACTIVE_SHIFT

/////////////////////////////////////////////////////////////////////////////////////////

#ifdef BT_USE_DOUBLE_PRECISION

    #define btglLoadMatrix  glLoadMatrixd
    #define btglMultMatrix  glMultMatrixd
    #define btglColor3      glColor3d
    #define btglVertex3     glVertex3d

#else

    #define btglLoadMatrix  glLoadMatrixf
    #define btglMultMatrix  glMultMatrixf
    #define btglColor3      glColor3f
    #define btglVertex3     glVertex3d

#endif

#endif // _GLUT_STUFF_H_INCLUDED
