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
    cin.ignore();
    getline(cin, input_str);

    return input_str;
}

vector<string> get_constraints() {
    cout << "How many constraints do you have?" << endl;
    int num_constraints;
    cin >> num_constraints;

    cout << "Enter the " << num_constraints << " constraints." << endl;
    cout << "Follow the rules for the maximisation goal, additionally..." << endl;
    cout << "Comparators only <= or >=; only constants in the RHS; no constants in the LHS." << endl;
    cout << "No negative numbers in the RHS. Multiply by -1 if necessary." << endl;
    cout << "A valid example: 9x + -8y <= 100" << endl;

    vector<string> constraints;
    for (int i = 0; i < num_constraints; i++) {
        string constraint; 
        cin.ignore();
        getline(cin, constraint);

        assert(constraint.find(" <= ") != string::npos || constraint.find(" >= ") != string::npos);

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

    // Must have + between terms everywhere. Turn 1x - 1y <= 0 into 1x + -1y <= 0.
    // Only integers
    // Shouldn't have uncollected variables like 2x + 1x <= 10. 
    // This includes constants.

    // TODO undo
    string goal_str = "9x + 2y + 4z";
    vector<string> constraints = {
        "1x + 1y <= 9", 
        "3x + 1y <= 18",
        "1x <= 7", 
        "1y <= 6", 
        "1z <= 11",
        "1x + 1y + 1z <= 10", 
        "1y + 2z <= 22",
        "1y >= 1",
        "1z >= 5",
        };
    // string goal_str = get_goal();
    // vector<string> constraints = get_constraints();

    LPsolver solver = LPsolver(goal_str, constraints);
    solver.solve();
    string results = solver.getResults();
    cout << results << endl;
}


// Benchmark the solver.
void benchmark() {
    // Explain what will be done


    // Generate random input


    // Benchmark, maybe against a brute-force approach?
    // Maybe there's a library that could be used?
}