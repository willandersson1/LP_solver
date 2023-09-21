#include <iostream>
#include <vector>
#include <string>

#include "headers/functionality.h"
#include "headers/LPsolver.h"

// Solve a general linear program.
void solveGeneralLP() {
    // Explain the required input structure...

    // Based off of https://en.wikipedia.org/wiki/Linear_programming
    // One LINEAR MINIMISATION goal, with no constants.
    // Unlimited LINEAR constraints
    // Constraints can't have variables not in the goal
    // No 0 coefficients
    // No unrestricted variables

    // Comparators only >=, <=, =
    // Must have + between terms everywhere. Turn x - y <= 0 into x + -y <= 0.
    // Only integers
    // Shouldn't have uncollected variables like 2x + x <= 10. 
    // This includes constants.
    // TODO: unless I parse to allow this

    // Get input
    printf("What function would you like to maximise?\n");



    // Check the input is valid... 


    // Do the computation...


    // Print result


}


// Benchmark the solver.
void benchmark() {
    // Explain what will be done


    // Generate random input


    // Benchmark, maybe against a brute-force approach?
    // Maybe there's a library that could be used?
}