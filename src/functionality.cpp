#include <iostream>
#include <vector>
#include <string>

#include "functionality.hpp"
#include "LPsolver.hpp"

// Solve a general linear program.
void solveGeneralLP() {
    // Explain the required input structure...

    // Based off of https://en.wikipedia.org/wiki/Linear_programming
    // One LINEAR maximisation goal
    // Unlimited LINEAR constraints
    // Constraints can't have variables not in the maximisation goal
    // No 0 coefficients
    // No unrestricted variables

    // Operators only >=, <=, =
    // Must have + between terms in goal
    // Only integers
    // Shouldn't have different variables in same line (ie 2x + x < y)
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