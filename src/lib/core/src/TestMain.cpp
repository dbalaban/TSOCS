
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

const long ITERATIONS = 100;

int main(int argc, char* argv[])
{

    long iterations = 0;
    long solvedonlya = 0, solvedonlyb = 0, solvedboth = 0;
    long failedonla = 0, failedonlyb, failedboth = 0;

    if (argc > 1)
        iterations = atol(argv[1]);
    else iterations = ITERATIONS;

    RandomProblemGenerator pgen(3,7);

    for (long i = 0; i < iterations; i++)
    {
        TOCORBA problem = pgen.generateProblem();
        ProblemSolver solver(problem);

        AngleFormulation aform;
        bool oka = solver.Solve((AbstractFormulation<AngleFormulation>*)(&aform), ProblemType::TSOCStype);

        BasicFormulation bform;
        bool okb = solver.Solve((AbstractFormulation<BasicFormulation>*)(&bform), ProblemType::TSOCStype);

        if (oka && okb)
            solvedboth++;
        else if (!oka && !okb)
            failedboth++;
        else if (oka)
        {
            solvedonlya++;
            failedonlyb++;
        }
        else if (okb)
        {
            solvedonlyb++;
            failedonla++;
        }
    }   

    // report results in tabular format
    std::cout << std::endl << "Results from " << iterations << " iterations:" << std::endl;
    std::cout << "\t\tSuccess\t\tFailure" << std::endl 
              << "\t\t-------\t\t-------" << std::endl;
    std::cout << "\tBasic:\t" << solvedonlyb << "\t\t" << failedonlyb << std::endl;
    std::cout << "\tAngle:\t" << solvedonlya << "\t\t" << failedonla  << std::endl;
    std::cout << "\tBoth:\t"  << solvedboth  << "\t\t" << failedboth  << std::endl << std::endl;

}