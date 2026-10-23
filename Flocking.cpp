// FILE: battle-formation flocking (implementation) - AA1, task 3.
// FITS IN: see Flocking.hpp for the structure and the Total War reference.
#include "Flocking.h"
#include "Engine/Agent.hpp"

FlockingParams FlockingParams::forTileSize(float tile)
{
    FlockingParams p;
    p.separationRadius = tile * 1.0f;   // preferred spacing between men
    p.alignmentRadius = tile * 3.0f;   // who they copy the heading of
    p.cohesionRadius = tile * 6.0f;   // large: the block must not split in two
    p.separationWeight = 1.6f;
    p.alignmentWeight = 0.6f;
    p.cohesionWeight = 0.4f;
    return p;
}

SteeringPtr makeFlocking(const FlockingParams& params)
{
    auto flock = std::make_shared<WeightedBlending>();
    flock->Add(std::make_shared<Separation>(params.separationRadius), params.separationWeight);
    flock->Add(std::make_shared<Alignment>(params.alignmentRadius), params.alignmentWeight);
    flock->Add(std::make_shared<Cohesion>(params.cohesionRadius), params.cohesionWeight);
    return flock;
}

ArmyParams ArmyParams::forTileSize(float tile)
{
    ArmyParams p;
    p.flocking = FlockingParams::forTileSize(tile);
    p.flockWeight = 1.0f;
    p.goalWeight = 1.0f;
    p.cruiseSpeedFactor = 0.7f;          // a march, not a sprint
    p.arrivalSlowRadius = tile * 4.0f;
    p.arrivalStopRadius = tile * 0.5f;
    p.arrivalSwitchRadius = tile * 5.0f;
    p.overlapRadius = tile * 0.45f;
    p.overlapEpsilon = 140.0f;
    return p;
}

namespace {

    // Shared skeleton of makeMarch / makeArrival: only the goal behaviour differs.
    SteeringPtr makeArmyBehaviour(const ArmyParams& params, SteeringPtr goal)
    {
        auto blend = std::make_shared<WeightedBlending>();
        blend->Add(makeFlocking(params.flocking), params.flockWeight);
        blend->Add(std::move(goal), params.goalWeight);

        auto root = std::make_shared<PrioritizedList>(params.overlapEpsilon);
        root->Add(std::make_shared<Separation>(params.overlapRadius));   // highest priority
        root->Add(blend);
        return root;
    }

} // namespace

SteeringPtr makeMarch(const ArmyParams& params)
{
    return makeArmyBehaviour(params, std::make_shared<Seek>(params.cruiseSpeedFactor));
}

SteeringPtr makeArrival(const ArmyParams& params)
{
    return makeArmyBehaviour(params, std::make_shared<Arrive>(params.arrivalSlowRadius,
        params.arrivalStopRadius,
        params.cruiseSpeedFactor));
}

ArmySteering ArmySteering::make(float tileSize)
{
    ArmyParams params = ArmyParams::forTileSize(tileSize);
    ArmySteering s;
    s.march = makeMarch(params);
    s.arrival = makeArrival(params);
    s.switchRadius = params.arrivalSwitchRadius;
    return s;
}

void ArmySteering::applyTo(Agent& agent) const
{
    agent.steering = march;
    agent.arrivalSteering = arrival;
    agent.arrivalRadius = switchRadius;
}
