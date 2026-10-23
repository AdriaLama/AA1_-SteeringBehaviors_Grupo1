// FILE: steering behaviour hierarchy (implementation) - AA1, task 2.
// FITS IN: see SteeringBehaviors.hpp for the contracts; this file is the "how".
#include "SteeringBehaviors.h"
#include "Engine/Agent.hpp"
#include "Engine/World.hpp"
#include <algorithm>

namespace {

    // How hard a velocity error is corrected, in 1/second: force = error * gain.
    // 4 means "close the gap in about a quarter of a second" - responsive enough
    // for soldiers but not a snap. The result is always clamped to maxForce.
    const float kVelocityGain = 4.0f;

    // Steering that makes the agent's velocity approach `desiredVelocity`.
    Steering velocityMatch(const Agent& agent, const Vector2& desiredVelocity)
    {
        return Steering(((desiredVelocity - agent.velocity) * kVelocityGain).truncated(agent.maxForce));
    }

} // namespace

// ---------------------------------------------------------------------------
// Leaves
// ---------------------------------------------------------------------------

Steering Seek::GetSteering(const Agent& agent, const World&) const
{
    if (!agent.hasTarget) {
        return Steering();
    }
    Vector2 desired = (agent.target - agent.position).normalized() * (agent.maxSpeed * speedFactor_);
    return velocityMatch(agent, desired);
}

Steering Arrive::GetSteering(const Agent& agent, const World&) const
{
    if (!agent.hasTarget) {
        return Steering();
    }
    Vector2 toTarget = agent.target - agent.position;
    float   dist = toTarget.length();

    // 0 at stopRadius (or closer), 1 at slowRadius (or farther), linear between.
    float span = std::max(slowRadius_ - stopRadius_, 1e-3f);
    float scale = std::min(std::max((dist - stopRadius_) / span, 0.0f), 1.0f);

    Vector2 desired = toTarget.normalized() * (agent.maxSpeed * speedFactor_ * scale);
    return velocityMatch(agent, desired);
}

Steering Separation::GetSteering(const Agent& agent, const World& world) const
{
    Vector2 push(0.0f, 0.0f);
    for (const Agent& other : world.agents) {
        if (&other == &agent) {
            continue;
        }
        Vector2 offset = agent.position - other.position;
        float   dist = offset.length();
        if (dist >= radius_) {
            continue;
        }
        Vector2 away;
        if (dist > 1e-4f) {
            away = offset / dist;
        }
        else {
            // Exactly on top of each other: no direction exists, so pick
            // opposite ones deterministically (by address) so the pair splits.
            away = (&agent < &other) ? Vector2(1.0f, 0.0f) : Vector2(-1.0f, 0.0f);
        }
        push += away * ((radius_ - dist) / radius_);
    }
    return Steering(push.truncated(1.0f) * agent.maxForce);
}

Steering Alignment::GetSteering(const Agent& agent, const World& world) const
{
    Vector2 sum(0.0f, 0.0f);
    int     count = 0;
    for (const Agent& other : world.agents) {
        if (&other == &agent) {
            continue;
        }
        if (distance(agent.position, other.position) < radius_) {
            sum += other.velocity;
            ++count;
        }
    }
    if (count == 0) {
        return Steering();
    }
    return velocityMatch(agent, sum / (float)count);
}

Steering Cohesion::GetSteering(const Agent& agent, const World& world) const
{
    Vector2 sum(0.0f, 0.0f);
    int     count = 0;
    for (const Agent& other : world.agents) {
        if (&other == &agent) {
            continue;
        }
        if (distance(agent.position, other.position) < radius_) {
            sum += other.position;
            ++count;
        }
    }
    if (count == 0) {
        return Steering();
    }
    Vector2 toCentre = sum / (float)count - agent.position;
    // Proportional pull: full force at `radius_` from the centre, none on it.
    return Steering((toCentre / radius_).truncated(1.0f) * agent.maxForce);
}

// ---------------------------------------------------------------------------
// Composite
// ---------------------------------------------------------------------------

void CompositeBehavior::Add(SteeringPtr behavior, float weight)
{
    if (!behavior || behavior.get() == this) {
        return;   // null, or a composite containing itself (infinite recursion)
    }
    children_.push_back({ std::move(behavior), weight });
}

bool CompositeBehavior::Remove(const SteeringPtr& behavior)
{
    auto newEnd = std::remove_if(children_.begin(), children_.end(),
        [&](const Child& c) { return c.behavior == behavior; });
    bool removed = newEnd != children_.end();
    children_.erase(newEnd, children_.end());
    return removed;
}

bool CompositeBehavior::SetWeight(const SteeringPtr& behavior, float weight)
{
    bool found = false;
    for (Child& c : children_) {
        if (c.behavior == behavior) {
            c.weight = weight;
            found = true;
        }
    }
    return found;
}

Steering WeightedBlending::GetSteering(const Agent& agent, const World& world) const
{
    Steering result;
    for (const Child& c : children_) {
        result += c.behavior->GetSteering(agent, world) * c.weight;
    }
    result.linear = result.linear.truncated(agent.maxForce);
    return result;
}

Steering PrioritizedList::GetSteering(const Agent& agent, const World& world) const
{
    for (const Child& c : children_) {
        Steering s = c.behavior->GetSteering(agent, world);
        if (!s.isZero(epsilon_)) {
            return s;
        }
    }
    return Steering();
}
