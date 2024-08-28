
// Main test loop for ceres solver.
// Generate random problems for 2 different formulations (angle, basic),
// iterate many times and track success/failure
// Author: Bob Balaban, August 2024

#include "angle_formulation.h"
#include "basic_formulation.h"
#include "random_problem_generator.h"
#include "TOC-ORBA.h"
#include "problem_solver.h"

#include <stdio.h>
#include <iostream>

const int ITERATIONS = 100;

int main(int argc, char* argv[])
{
    long bsuccesses = 0, asuccesses = 0;
    long bfailures = 0,  afailures = 0;

    RandomProblemGenerator pgen(3,9);

    for (long i = 0; i < ITERATIONS; i++)
    {
        TOCORBA problem = pgen.generateProblem();
        ProblemSolver *solver = new ProblemSolver(problem);

        AngleFormulation aform;
        bool ok = solver->Solve((AbstractFormulation<AngleFormulation>*)(&aform));
        if (ok)
            asuccesses++;
        else afailures++;

        BasicFormulation bform;
        ok = solver->Solve((AbstractFormulation<BasicFormulation>*)(&bform));
        if (ok)
            bsuccesses++;
        else bfailures++;

        delete solver;
    }   

    std::cout << std::endl << "Results of " << ITERATIONS << " iterations (BasicFormulations): " << bsuccesses << " success, " << bfailures << " failed" << std::endl;
    std::cout << "Results of " << ITERATIONS << " iterations (AngleFormulations): " << asuccesses << " success, " << afailures << " failed" << std::endl;
    
}