#include <assert.h>
#include <iostream>
#include <vector>
#include <string>

#include "headers/functionality.h"
#include "headers/LPsolver.h"

using namespace std;

string get_goal() {
    cout << "Input the maximisation goal." << endl;
    cout << "There should be no constants, it must be linear, variables must be a single alphabetical character" << endl;
    cout << "Coefficients must be non-zero integers; terms separated by \" + \"." << endl;
    cout << "There can be no uncollected variables like \"2x + x\"" << endl;
    cout << "An example: 4x + -5y + 1z." << endl;

    string input_str;
    input_str = "2x"; // TODO undo
    // cin.ignore();
    // getline(cin, input_str);

    return input_str;
}

vector<string> get_constraints() {
    cout << "How many constraints do you have?" << endl;
    int num_constraints;
    num_constraints = 1; // TODO undo
    // cin >> num_constraints;

    cout << "Enter the " << num_constraints << " constraints." << endl;
    cout << "Follow the rules for the maximisation goal, additionally..." << endl;
    cout << "Comparators only <=; only constants in the RHS; no constants in the LHS." << endl;
    cout << "An example: 9x + -8y <= 100" << endl;

    vector<string> constraints; 
    for (int i = 0; i < num_constraints; i++) {
        string constraint; 
        constraint = "5x <= 500"; // TODO undo
        // cin.ignore();
        // getline(cin, constraint);

        assert(constraint.find(" <= ") != string::npos);
        assert(constraint.find(">") == string::npos);

        constraints.push_back(constraint);
    }

    return constraints;
}

// Solve a general linear program.
void solveGeneralLP() {
    // Explain the required input structure...

    // Based off of https://en.wikipedia.org/wiki/Linear_programming
    // One LINEAR MAXIMISATION goal, with no constants.
    // Unlimited LINEAR constraints
    // Constraints can't have variables not in the goal
    // No 0 coefficients
    // No unrestricted variables

    // Comparators only <=
    // Must have + between terms everywhere. Turn x - y <= 0 into x + -y <= 0.
    // Only integers
    // Shouldn't have uncollected variables like 2x + x <= 10. 
    // This includes constants.

    string goal_str = get_goal();
    goal_str = "1x"; // TODO undo
    vector<string> constraints = get_constraints();

    LPsolver solver = LPsolver(goal_str, constraints);
    vector<string> result = solver.solve();

    // Print result
    // TODO
}


// Benchmark the solver.
void benchmark() {
    // Explain what will be done


    // Generate random input


    // Benchmark, maybe against a brute-force approach?
    // Maybe there's a library that could be used?
}