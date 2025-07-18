#include "hta/ai/Vehicle.h"
#include "fix/autobrakefix.hpp"
#include "routines.hpp"

namespace kraken::fix::autobrakefix {
    void __fastcall SetThrottle(ai::Vehicle* vehicle, int, float throttle, bool autobrake)
    {
        if (vehicle->m_pPath) {
            if (vehicle->m_pathNum < vehicle->m_pPath->m_size - 1) {
                CVector velocity;
                vehicle->GetLinearVelocity(&velocity);

                float speedSquared = velocity.dot(velocity);

                if (speedSquared > 1e-10f) {
                    CVector forward;
                    vehicle->GetDirection(&forward);
                    float dot = forward.dot(velocity);
                    if (dot > 0.0f) {
                        vehicle->SetThrottle(throttle, false);
                        return;
                    }
                }
            }
        }
        vehicle->SetThrottle(throttle, autobrake);
    }

    void Apply()
    {
        kraken::routines::make_call((void*) 0x005D3137, (void*) &SetThrottle);
    }
}