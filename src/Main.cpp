/**
 *  @file      Main.cpp
 *  @brief     The main entry point for the stand-alone GLUT application.
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-06-11
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "PassiveWalkerApplication.h"
#include "GLUT_Framework.h"

/////////////////////////////////////////////////////////////////////////////////////////

/** Definition of the static instance wrapper for our GLUT application.
 */
template<class T> T* GLUT_Framework<T>::Application = 0;

/////////////////////////////////////////////////////////////////////////////////////////

/** The main entry point for the stand-alone GLUT application.
 * Creates an instance of the PassiveWalkerApplication class and runs glutMainLoop().
 * See GLUT_Framework<>.
 */
int main( int argc, char* argv[] )
{
    return GLUT_Framework<PassiveWalkerApplication>::Main( argc, argv );
}

/////////////////////////////////////////////////////////////////////////////////////////

// Declare required libraries (on windows)
//
#if defined(_WIN32) && defined(_MSC_VER)
    #ifdef _DEBUG
        #define LIB_EXT "_debug.lib"
    #else
        #define LIB_EXT ".lib"
    #endif
    #pragma comment( lib, "BulletDynamics"  LIB_EXT )
    #pragma comment( lib, "BulletCollision" LIB_EXT )
    #pragma comment( lib, "LinearMath"      LIB_EXT )
#endif
