//
// Created by Sofia Iannicelli on 11/13/24.
//

#include "Simulator.h"

#include <iostream>

Simulator::Simulator(int fps) {

}

Simulator::~Simulator()
{

}

void Simulator::run(int frames) {
    double ts;
    double curTime = 0.0;

    // currently done in constructor:
    // 1. Precompute wij_
    // 2. Set quasistatic material frame

    for(int frame = 0; frame <= frames; ++frame) {
        std::cout << "Current frame: " << frame << std::endl;

        while(curTime < frame) {
            std::cout << "\t current time: " << curTime << std::endl;

            // TODO: get time step to take
            ts = 0.1;

            // STEPS
            // [RB] Apply torque to rigid body
            // [RB] Integrate rigid-body

            // Compute forces on centerline


            // Integrate centerline

            // Enforce inextensibility and [RB] rigid body coupling

            // Collision detection and response

            // Update bishop frame

            // Update quasistatic material frame

            curTime += ts;
        }
    }
}

