/**
 *  @file      Constants.cpp
 *  @brief     Implementation of the Const class that encapsulates various
 *             physical and numerical constants.
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Constants.h"

#include <limits> // std::numeric_limits

/////////////////////////////////////////////////////////////////////////////////////////

const btVector3 Const::g_n( 0, btScalar( -9.80665 ), 0 ); // Standard gravity

/////////////////////////////////////////////////////////////////////////////////////////

const btVector3 Const::Zero ( 0, 0, 0 );

const btVector3 Const::X ( 1, 0, 0 );
const btVector3 Const::Y ( 0, 1, 0 );
const btVector3 Const::Z ( 0, 0, 1 );

/////////////////////////////////////////////////////////////////////////////////////////

#ifdef BT_USE_DOUBLE_PRECISION

    const btScalar Const::Max = std::numeric_limits<double>::max ();
    const btScalar Const::Min = std::numeric_limits<double>::min ();
    const btScalar Const::Eps = std::numeric_limits<double>::epsilon ();
    const btScalar Const::Inf = std::numeric_limits<double>::infinity ();
    const btScalar Const::NaN = std::numeric_limits<double>::quiet_NaN ();

#else

    const btScalar Const::Max = std::numeric_limits<float>::max ();
    const btScalar Const::Min = std::numeric_limits<float>::min ();
    const btScalar Const::Eps = std::numeric_limits<float>::epsilon ();
    const btScalar Const::Inf = std::numeric_limits<float>::infinity ();
    const btScalar Const::NaN = std::numeric_limits<float>::quiet_NaN ();

#endif
