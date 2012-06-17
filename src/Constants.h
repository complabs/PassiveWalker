#pragma once
#ifndef _CONSTANTS_H_INCLUDED
#define _CONSTANTS_H_INCLUDED

/**
 *  @file      Constants.h
 *  @brief     Definitions for the Const static class that encapsulates various
 *             physical and numerical constants.
 *  @author    Mikica Kocic
 *  @version   0.2
 *  @date      2012-06-10
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////
// Referenced BT classes

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

/////////////////////////////////////////////////////////////////////////////////////////
/** @class Const
 *
 * Physical, mathematical and numerical constants.
 */
class Const
{
public:

    /////////////////////////////////////////////////////////////////////////////////////

    /** Gets standard gravity (standard acceleration due to free fall in kg m/s^2).
     * @note Standard gravity is given along Y-axis as vertical axis.
     */
    const static btVector3 g_n;

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents zero vector.
     */
    const static btVector3 Zero;

    /** Represents X-axis unit vector.
     */
    const static btVector3 X;

    /** Represents Y-axis unit vector.
     */
    const static btVector3 Y;

    /** Represents Z-axis unit vector.
     */
    const static btVector3 Z;

    /////////////////////////////////////////////////////////////////////////////////////

    /** Represents the maximum finite value for a floating-point number.
     */
    const static btScalar Max;

    /** Represents the minimum finite value for a floating-point number.
     */
    const static btScalar Min;

    /** Gets the smallest positive `x` such that `x + Eps + x` is representable.
     */
    const static btScalar Eps;    

    /** Returns the representation of positive infinity.
     */
    const static btScalar Inf;

    /** Returns the representation of a quiet not a number (NaN).
     */
    const static btScalar NaN;

    /** Returns true if the argument is NaN.
     */
    static bool IsNaN( btScalar x )
    {
        return x != x;
    }

    /** Returns +1/-1 if the argument is positive/negative infinity.
     */
    static int IsInf( btScalar x )
    {
        btScalar delta = x - x;
        if ( x == x && delta != 0.0 ) {
            return x < 0.0 ? -1 : 1;
        }
        return 0;
    }

    /////////////////////////////////////////////////////////////////////////////////////
};

#endif // _CONSTANTS_H_INCLUDED
