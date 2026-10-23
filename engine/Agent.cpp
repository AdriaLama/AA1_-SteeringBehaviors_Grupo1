// FILE: Agent's movement (kinematics) + steering-behaviour plumbing (implementation).
// FITS IN: World::update calls selectSteering/computeSteering/integrate every
// frame; WHAT the agent wants comes from its SteeringBehavior (see
// SteeringBehaviors.hpp / Flocking.hpp), this file only applies it.
// EXTEND: new movement belongs in a SteeringBehavior, not here.
#include "Agent.hpp"
#include "World.hpp"
#include <algorithm>
#include <cmath>

namespace {

    // Summary: whether a body of `radius`, centred at `center`, is clear of
    // blocked tiles. Inputs: world (read-only), center/radius in world units.
    // Output: bool; no side effects. Errors: none.
    // WATCH OUT: checked at 4 edge points, not the whole circle, so the agent
    // can't corner-cut through a blocked tile's edge on a diagonal move - don't
    // reduce these points without re-checking that guarantee still holds.
    bool canOccupy(const World& world, const Vector2& center, float radius)
    {
        const Vector2 offsets[4] = {
            Vector2(radius, 0.0f), Vector2(-radius, 0.0f),
            Vector2(0.0f, radius), Vector2(0.0f, -radius),
        };
        for (const Vector2& off : offsets) {
            Vector2 p = center + off;
            int tx, ty;
            world.worldToTile(p, tx, ty);
            if (!world.isWalkable(tx, ty)) {
                return false;
            }
        }
        return true;
    }


    // Below this speed AND this fraction of maxForce an agent is "settled": its
    // velocity is zeroed so a crowd that has reached its destination stands still
    // instead of shimmering from tiny opposing separation/cohesion forces.
    const float kRestSpeed = 6.0f;
    const float kRestForceFraction = 0.05f;

} // namespace

void Agent::selectSteering()
{
    if (!arrivalSteering || arrivalRadius <= 0.0f || !hasTarget) {
        arriving_ = false;
        return;
    }
    float dist = distance(position, target);
    if (arriving_) {
        if (dist > arrivalRadius * 1.25f) {
            arriving_ = false;
        }
    }
    else if (dist < arrivalRadius) {
        arriving_ = true;
    }
}

Vector2 Agent::computeSteering(const World& world) const
{
    const SteeringPtr& active = arriving_ ? arrivalSteering : steering;
    if (!active) {
        return Vector2(0.0f, 0.0f);
    }
    return active->GetSteering(*this, world).linear.truncated(maxForce);
}

void Agent::integrate(float dt, const World& world, const Vector2& force)
{
    Vector2 acceleration = force.truncated(maxForce);
    velocity = (velocity + acceleration * dt).truncated(maxSpeed);

    if (velocity.lengthSquared() < kRestSpeed * kRestSpeed &&
        acceleration.lengthSquared() < (kRestForceFraction * maxForce) * (kRestForceFraction * maxForce)) {
        velocity = Vector2(0.0f, 0.0f);
        return;
    }

    // Move one axis at a time so the agent slides along a blocked tile's edge
    // instead of stopping dead; a blocked axis loses its velocity.
    Vector2 delta = velocity * dt;
    Vector2 candidate = position;
    candidate.x += delta.x;
    if (canOccupy(world, candidate, radius)) position.x = candidate.x;
    else velocity.x = 0.0f;
    candidate = position;
    candidate.y += delta.y;
    if (canOccupy(world, candidate, radius)) position.y = candidate.y;
    else velocity.y = 0.0f;
    if (velocity.lengthSquared() > 1e-4f) heading = std::atan2(velocity.y, velocity.x);
}

void Agent::update(float dt, const World& world)
{
    selectSteering();
    integrate(dt, world, computeSteering(world));
}

// See Agent.hpp for the full contract (initInfluence/setInfluence/influenceAt/
// clearInfluence) - implementation notes only below.

void Agent::initInfluence(const World& world)
{
    influenceWidth_ = world.widthTiles;
    influenceHeight_ = world.heightTiles;
    influence_.assign((size_t)influenceWidth_ * influenceHeight_, 0.0f);
}

void Agent::setInfluence(int tileX, int tileY, float value)
{
    if (tileX < 0 || tileY < 0 || tileX >= influenceWidth_ || tileY >= influenceHeight_) {
        return;
    }
    influence_[(size_t)tileY * influenceWidth_ + tileX] = value;
}

float Agent::influenceAt(int tileX, int tileY) const
{
    if (tileX < 0 || tileY < 0 || tileX >= influenceWidth_ || tileY >= influenceHeight_) {
        return 0.0f;
    }
    return influence_[(size_t)tileY * influenceWidth_ + tileX];
}

void Agent::clearInfluence()
{
    std::fill(influence_.begin(), influence_.end(), 0.0f);
}
