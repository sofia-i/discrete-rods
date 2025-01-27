//
// Created by Sofia Iannicelli on 11/13/24.
//

#ifndef DISCRETE_RODS_SIMULATOR_H
#define DISCRETE_RODS_SIMULATOR_H

#include <vector>
#include "Rod.h"

class Simulator {

public:
    Simulator(int fps);
    virtual ~Simulator();

    void run(int frames);

private:
    std::vector<Rod*> rods;

};


#endif //DISCRETE_RODS_SIMULATOR_H
