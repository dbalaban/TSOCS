
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
    long successes = 0;
    long failures = 0;

    RandomProblemGenerator pgen(3,9);

    for (long i = 0; i < ITERATIONS; i++)
    {
        TOCORBA problem = pgen.generateProblem();
        ProblemSolver *solver = new ProblemSolver(problem);

        AngleFormulation *form = new AngleFormulation();
        bool ok = solver->Solve(form);

        if (ok)
            successes++;
        else failures++;

        delete form;
        delete solver;
    }   

    std::cout << "Results of " << ITERATIONS << " iterations: " << successes << " success, " << failures << " failed";
    
}