#pragma once
#ifndef _RAG_DOLL_H_INCLUDED
#define _RAG_DOLL_H_INCLUDED

/**
 *  @file      RagDoll.h
 *  @brief     Definitions for the RagDoll class that encapsulates a simple rag-doll.
 *  @author    Mikica Kocic
 *  @version   0.1
 *  @date      2012-06-02
 *  @copyright GNU Public License.
 */

/////////////////////////////////////////////////////////////////////////////////////////

#include "Machine.h"

/////////////////////////////////////////////////////////////////////////////////////////

/** Represents a simple rag doll that demonstrates a usage for the Machine class.
 * The RagDoll class is used to instantiate 'shooting boxes' in this application.
 */
class RagDoll : public Machine
{
private: // parts :)

    Machine::Element Pelvis;
    Machine::Element UpperTrunk;
    Machine::Element Head;
    Machine::Element LeftThigh;
    Machine::Element LeftShank;
    Machine::Element LeftUpperArm;
    Machine::Element LeftForearm;
    Machine::Element RightThigh;
    Machine::Element RightShank;
    Machine::Element RightUpperArm;
    Machine::Element RightForearm;

    Machine::Joint UpperTrunkHead;
    Machine::Joint PelvisUpperTrunk;
    Machine::Joint LeftHip;
    Machine::Joint LeftKnee;
    Machine::Joint LeftShoulder;
    Machine::Joint LeftElbow;
    Machine::Joint RightHip;
    Machine::Joint RightKnee;
    Machine::Joint RightShoulder;
    Machine::Joint RightElbow;

public:

    RagDoll(
        btDynamicsWorld* ownerWorld, 
        const btTransform& offset,
        btScalar mass    = btScalar(1),
        bool rigidJoints = false,
        btScalar scale   = btScalar(1)
    );
};

#endif // _RAG_DOLL_H_INCLUDED
