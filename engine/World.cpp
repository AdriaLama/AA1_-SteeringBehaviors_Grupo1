// FILE: World's tile-grid queries + agent-list simulation step (implementation).
// FITS IN: see World.hpp for the full per-method contracts; this file is the
// "how", not the "what/why". EXTEND: World.hpp's header explains where to
// add another shared per-tile layer.
#include "World.hpp"

World::World(int widthTiles, int heightTiles, int tileSize)
    : widthTiles(widthTiles), heightTiles(heightTiles), tileSize(tileSize)
{
    if (widthTiles > 0 && heightTiles > 0) {
        blocked_.assign((size_t)widthTiles * heightTiles, 0);
    }
}

bool World::inBounds(int tileX, int tileY) const
{
    return tileX >= 0 && tileY >= 0 && tileX < widthTiles && tileY < heightTiles;
}

bool World::isWalkable(int tileX, int tileY) const
{
    if (!inBounds(tileX, tileY)) {
        return false;
    }
    return blocked_[index(tileX, tileY)] == 0;
}

void World::setBlocked(int tileX, int tileY, bool blocked)
{
    if (!inBounds(tileX, tileY)) {
        return;
    }
    blocked_[index(tileX, tileY)] = blocked ? 1 : 0;
}

Vector2 World::tileCenter(int tileX, int tileY) const
{
    return Vector2((tileX + 0.5f) * tileSize, (tileY + 0.5f) * tileSize);
}

void World::worldToTile(const Vector2& p, int& tileX, int& tileY) const
{
    // WATCH OUT: (int) truncates toward zero, not floor - see World.hpp.
    tileX = (tileSize > 0) ? (int)(p.x / tileSize) : 0;
    tileY = (tileSize > 0) ? (int)(p.y / tileSize) : 0;
}

void World::addObject(ObjectType type, int tileX, int tileY, int tileIndex)
{
    if (!inBounds(tileX, tileY)) {
        return;
    }
    WorldObject obj;
    obj.type = type;
    obj.tileX = tileX;
    obj.tileY = tileY;
    obj.position = tileCenter(tileX, tileY);
    obj.tileIndex = tileIndex;
    objects.push_back(obj);
    setBlocked(tileX, tileY, true);
}

void World::update(float dt)
{
    // Agents without their own behaviour get the shared army one. Built once,
    // on first use, so it picks up the tileSize the map was loaded with.
    if (!armyReady_ && tileSize > 0) {
        armySteering_ = ArmySteering::make((float)tileSize);
        armyReady_ = true;
    }
    if (armyReady_) {
        for (Agent& agent : agents) {
            if (!agent.steering) {
                armySteering_.applyTo(agent);
            }
        }
    }

    // Pass 1: every agent decides from the same snapshot. Pass 2: all move.
    forces_.resize(agents.size());
    for (size_t i = 0; i < agents.size(); ++i) {
        agents[i].selectSteering();
        forces_[i] = agents[i].computeSteering(*this);
    }
    for (size_t i = 0; i < agents.size(); ++i) {
        agents[i].integrate(dt, *this, forces_[i]);
    }
}
