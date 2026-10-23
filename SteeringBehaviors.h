#ifndef ENGINE_STEERING_BEHAVIORS_HPP
#define ENGINE_STEERING_BEHAVIORS_HPP
#include <cstddef>
#include <memory>
#include <vector>
#include "Engine/Vector2.hpp"

class Agent;   // forward declarations: Agent.hpp includes this header
class World;

// FILE: steering behaviour hierarchy (Composite pattern) - AA1, task 2.
// FITS IN: Agent owns a SteeringPtr and, every frame, asks it for a
// Steering (World::update -> Agent::computeSteering). Leaves (Seek, Arrive,
// Separation, Alignment, Cohesion) and composites (WeightedBlending,
// PrioritizedList) share ONE interface, so a composite can hold leaves or
// other composites, and any of them can be swapped on an agent at runtime
// (Agent::steering = std::make_shared<Arrive>(...)).
// IMPORTANT INVARIANT: GetSteering is const and stateless with respect to the
// agent it is asked about, so ONE behaviour object can be shared by every
// agent in the army (see Flocking.hpp). Forces are accelerations
// (units / second^2) and every leaf returns at most agent.maxForce.
// EXTEND: new behaviour = new leaf deriving from SteeringBehavior; it can be
// dropped into any composite without touching the others.

// Output of a behaviour. Only a linear acceleration for now: heading is
// derived from velocity by Agent, so no angular part is needed.
struct Steering
{
    Vector2 linear;   // desired acceleration, units / second^2

    Steering() = default;
    explicit Steering(const Vector2& linear) : linear(linear) {}

    Steering  operator+(const Steering& o) const { return Steering(linear + o.linear); }
    Steering& operator+=(const Steering& o) { linear += o.linear; return *this; }
    Steering  operator*(float s) const { return Steering(linear * s); }
    bool      isZero(float epsilon) const { return linear.lengthSquared() <= epsilon * epsilon; }
};

// Component of the Composite pattern (UML: SteeringBehavior).
class SteeringBehavior
{
public:
    virtual ~SteeringBehavior() = default;
    // Summary: the acceleration this behaviour wants for `agent` right now.
    // Inputs: agent (the one being steered), world (read-only: neighbours).
    // Output: Steering; no side effects. Errors: none - a behaviour that has
    // nothing to do (no target, no neighbours) returns a zero Steering.
    virtual Steering GetSteering(const Agent& agent, const World& world) const = 0;
};
using SteeringPtr = std::shared_ptr<SteeringBehavior>;

// ---------------------------------------------------------------------------
// Leaves
// ---------------------------------------------------------------------------

// Reynolds seek: head at full speed straight at agent.target.
// speedFactor scales the cruise speed (1 = agent.maxSpeed). Zero without target.
class Seek final : public SteeringBehavior
{
public:
    explicit Seek(float speedFactor = 1.0f) : speedFactor_(speedFactor) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float speedFactor_;
};

// Reynolds arrive: like Seek, but the desired speed drops linearly to zero as
// the agent gets from slowRadius down to stopRadius from agent.target, so it
// stops on the target instead of overshooting. Zero without target.
class Arrive final : public SteeringBehavior
{
public:
    Arrive(float slowRadius, float stopRadius = 0.0f, float speedFactor = 1.0f)
        : slowRadius_(slowRadius), stopRadius_(stopRadius), speedFactor_(speedFactor) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float slowRadius_;
    float stopRadius_;
    float speedFactor_;
};

// Reynolds separation: push away from every neighbour closer than `radius`,
// the closer the harder (linear falloff, full force when overlapping).
class Separation final : public SteeringBehavior
{
public:
    explicit Separation(float radius) : radius_(radius) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float radius_;
};

// Reynolds alignment: match the average velocity of the neighbours within
// `radius`. Zero when alone.
class Alignment final : public SteeringBehavior
{
public:
    explicit Alignment(float radius) : radius_(radius) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float radius_;
};

// Reynolds cohesion: move towards the centre of mass of the neighbours within
// `radius`. The pull grows with the distance to that centre, so it vanishes
// at the centre (no oscillation around it). Zero when alone.
class Cohesion final : public SteeringBehavior
{
public:
    explicit Cohesion(float radius) : radius_(radius) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float radius_;
};

// ---------------------------------------------------------------------------
// Composite
// ---------------------------------------------------------------------------

// Composite of the pattern (UML: CompositeBehavior). Holds the children and
// Add/Remove; how their results are combined is up to the two subclasses.
class CompositeBehavior : public SteeringBehavior
{
public:
    struct Child
    {
        SteeringPtr behavior;
        float       weight;   // used by WeightedBlending, ignored by PrioritizedList
    };

    // Adds a child (null and self are ignored). Order = insertion order.
    void Add(SteeringPtr behavior, float weight = 1.0f);
    // Removes every occurrence of `behavior`. Output: true if something went.
    bool Remove(const SteeringPtr& behavior);
    // Retunes a child's weight at runtime. Output: false if it isn't a child.
    bool SetWeight(const SteeringPtr& behavior, float weight);
    void   Clear() { children_.clear(); }
    size_t Size() const { return children_.size(); }

protected:
    std::vector<Child> children_;
};

// Weighted Blending: sum of every child's steering times its weight,
// truncated to agent.maxForce so a nested composite stays bounded too.
class WeightedBlending final : public CompositeBehavior
{
public:
    Steering GetSteering(const Agent& agent, const World& world) const override;
};

// Prioritized List: children are asked in insertion order (first added =
// highest priority); the first one whose steering is bigger than `epsilon`
// (units / second^2) wins and the rest are not even evaluated. Use it for
// "emergency first, ordinary movement otherwise".
class PrioritizedList final : public CompositeBehavior
{
public:
    explicit PrioritizedList(float epsilon = 1.0f) : epsilon_(epsilon) {}
    Steering GetSteering(const Agent& agent, const World& world) const override;
private:
    float epsilon_;
};

#endif // ENGINE_STEERING_BEHAVIORS_HPP
