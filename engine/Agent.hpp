#ifndef ENGINE_AGENT_HPP
#define ENGINE_AGENT_HPP
#include <vector>
#include "Vector2.hpp"
#include "SteeringBehaviors.h"

class World; // forward declaration (Agent.cpp includes World.hpp)

// FILE: a moving agent - the thing students' AI actually drives.
// FITS IN: pure kinematics/AI, no SDL/rendering/isometric maths (Game draws it).
// The agent no longer decides how to move by itself: it holds a steering
// behaviour (SteeringBehaviors.hpp) and just integrates whatever acceleration
// that behaviour asks for. Swap `steering` at any time to change how it moves.
// EXTEND: new movement = a new SteeringBehavior (or a composite of existing
// ones) assigned to `steering`; Agent itself should not need to change.
class Agent
{
public:
    Vector2 position;                 // world-space position
    Vector2 velocity;                 // world-space velocity (units / second)
    // WATCH OUT: tuned relative to World::tileSize (currently 64 units/tile).
    // Keep these proportional to tileSize so behaviour (tiles/second, tiles
    // of radius) stays the same if the tile size ever changes again.
    float   maxSpeed = 400.0f;        // maximum speed (units / second)
    float   maxForce = 2800.0f;       // maximum steering acceleration (units / second^2)
    float   radius = 16.0f;         // body radius, for later collision / arrival
    float   heading = 0.0f;          // facing angle in radians (derived from velocity)

    // Current steering target in world-space, and whether one is set. Each
    // agent's own - not shared - so e.g. Game can point different agents at
    // different tiles (see Game::updateMouseTarget). Seek and Arrive read it.
    Vector2 target;
    bool    hasTarget = false;

    // --- Steering behaviours (task 2) -------------------------------------
    // The behaviour driving this agent. Left null, World::update gives the
    // agent the shared army behaviour (Flocking.hpp). Behaviours are stateless
    // per agent, so the same object may be shared by many agents.
    SteeringPtr steering;
    // Optional dynamic switch: while the agent is closer than arrivalRadius to
    // its target, arrivalSteering is used instead of `steering` (e.g. Seek ->
    // Arrive as the army nears the pointer). Not set / radius <= 0 = never.
    SteeringPtr arrivalSteering;
    float       arrivalRadius = 0.0f;

    // Summary: decide which behaviour is active this frame (steering vs
    // arrivalSteering). Hysteresis: it switches to arrival at arrivalRadius but
    // only goes back at 1.25 * arrivalRadius, so an agent hovering at the edge
    // doesn't flicker between the two. Side effects: updates the private flag.
    void selectSteering();
    // Summary: the acceleration the active behaviour asks for, clamped to
    // maxForce. Output: force (units / second^2); no side effects. A null
    // behaviour returns (0,0).
    Vector2 computeSteering(const World& world) const;
    // Summary: apply `force` for dt seconds - velocity, position (with tile
    // collision) and heading. Side effects: mutates this agent only.
    // WATCH OUT: reads other agents only through computeSteering, so call
    // computeSteering for ALL agents first and integrate afterwards (as
    // World::update does) - otherwise agents see half-updated neighbours.
    void integrate(float dt, const World& world, const Vector2& force);
    // Summary: convenience for a single agent: selectSteering + computeSteering
    // + integrate. World::update does NOT use this (it needs two passes).
    void update(float dt, const World& world);

    // --- Influence map (per-agent scalar field over the tile grid) --------
    // Summary: a per-agent per-tile float layer for AI code to read/write:
    // influence, a cost field, a visited-heatmap, whatever an exercise needs.
    // IMPORTANT INVARIANT: call initInfluence() once (sized to World's tile
    // grid) before set/at - skip it and the map silently stays 0x0, so
    // setInfluence/influenceAt just no-op/return 0 instead of crashing.
    // EXTEND: add more layers the same way (another vector + width/height)
    // if an exercise needs, say, a cost field alongside a visited-heatmap.
    void  initInfluence(const World& world);
    void  setInfluence(int tileX, int tileY, float value);
    float influenceAt(int tileX, int tileY) const;   // 0 if out of bounds or uninitialised
    void  clearInfluence();

private:
    bool               arriving_ = false;   // true while arrivalSteering is the active one

    int                influenceWidth_ = 0;
    int                influenceHeight_ = 0;
    std::vector<float> influence_;
};

#endif // ENGINE_AGENT_HPP