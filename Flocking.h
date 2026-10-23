#ifndef ENGINE_FLOCKING_HPP
#define ENGINE_FLOCKING_HPP
#include "SteeringBehaviors.h"

class Agent;

// FILE: battle-formation flocking built on the task-2 hierarchy - AA1, task 3.
// FITS IN: World::update hands every agent that has no behaviour of its own
// the ArmySteering below, and Agent switches between its two behaviours as it
// nears its target (see Agent::selectSteering). Everything here is composition
// of SteeringBehaviors.hpp classes - no new steering maths.
//
// Reference: Total War. A regiment moves as one compact block - men keep an
// even spacing (Separation), face and march the same way at a shared pace
// (Alignment), stragglers close up on the block (Cohesion), the whole block
// heads for the clicked point at a walking pace and slows down together when
// it gets there (Seek -> Arrive).
//
// The tree built by makeMarch (makeArrival is identical with Arrive for Seek):
//   PrioritizedList
//     1. Separation(overlapRadius)        emergency: only bodies that overlap
//     2. WeightedBlending
//          WeightedBlending (flocking)    Separation + Alignment + Cohesion
//          Seek                           the common objective (agent.target)
//
// EXTEND: retune the army by editing the numbers in FlockingParams::forTileSize
// / ArmyParams::forTileSize (Flocking.cpp); radii are in tiles so they follow
// World::tileSize.

struct FlockingParams
{
    float separationRadius;   // world units
    float alignmentRadius;
    float cohesionRadius;
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;

    static FlockingParams forTileSize(float tileSize);
};

// Separation + Alignment + Cohesion as a WeightedBlending (the task-3 flock).
SteeringPtr makeFlocking(const FlockingParams& params);

struct ArmyParams
{
    FlockingParams flocking;
    float flockWeight;         // weight of the whole flock in the final blend
    float goalWeight;          // weight of Seek / Arrive in the final blend
    float cruiseSpeedFactor;   // fraction of agent.maxSpeed the army marches at
    float arrivalSlowRadius;   // Arrive starts braking here
    float arrivalStopRadius;   // Arrive wants zero speed inside this
    float arrivalSwitchRadius; // distance to target where Seek is swapped for Arrive
    float overlapRadius;       // emergency separation range (bodies touching)
    float overlapEpsilon;      // emergency wins when its force exceeds this (u/s^2)

    static ArmyParams forTileSize(float tileSize);
};

// Flock + Seek (marching towards the target).
SteeringPtr makeMarch(const ArmyParams& params);
// Flock + Arrive (closing in on the target and stopping on it).
SteeringPtr makeArrival(const ArmyParams& params);

// The pair of behaviours an army agent swaps between. One instance is meant
// to be shared by the whole army (behaviours are stateless per agent).
struct ArmySteering
{
    SteeringPtr march;
    SteeringPtr arrival;
    float       switchRadius = 0.0f;

    static ArmySteering make(float tileSize);
    // Gives `agent` these behaviours (its previous ones are replaced).
    void applyTo(Agent& agent) const;
};

#endif // ENGINE_FLOCKING_HPP