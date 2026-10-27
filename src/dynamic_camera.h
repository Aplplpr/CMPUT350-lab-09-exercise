#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        // Finally update cameraUnadjustedCenter, but without lookahead.

        // 1. Calculate lookahead based on player velocity
        float targetLookahead = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            targetLookahead = camera.windowPercentToWorldDist(
                {lookAhead, 0.f}).x;
        } else if (trackedVelocityWorld.x < 0.f) {
            targetLookahead = camera.windowPercentToWorldDist(
                {-lookAhead, 0.f}).x;
        }

        // 2. Smoothly update lookahead using EMA
        lookaheadState += alpha * (targetLookahead - lookaheadState);

        // 3. Find player position relative to camera without lookahead
        float playerX = trackedPosWorld.x - cameraUnadjustedCenter.x;

        // 4. Convert zone boundaries into world distances
        
        float leftDead = camera.windowPercentToWorldDist({deadZoneX.min - 0.5f, 0.f}).x;
        float rightDead = camera.windowPercentToWorldDist({deadZoneX.max - 0.5f, 0.f}).x;
        float leftHard = camera.windowPercentToWorldDist({softZoneX.min - 0.5f, 0.f}).x;
        float rightHard = camera.windowPercentToWorldDist({softZoneX.max - 0.5f, 0.f}).x;


        // 5. Calculate camera displacement without lookahead
        float delta = 0.f;

        if (playerX < leftHard) {
            delta = playerX - leftHard;
        } else if (playerX > rightHard) {
            delta = playerX - rightHard;
        } else if (playerX < leftDead) {
            delta = alpha * (playerX - leftDead);
        } else if (playerX > rightDead) {
            delta = alpha * (playerX - rightDead);
        }

        // 6. Update camera center with lookahead
        cameraUnadjustedCenter.x += delta;

        sf::Vector2f newCenter = cameraUnadjustedCenter;
        newCenter.x += lookaheadState;
        camera.setCameraCenter(newCenter);
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
