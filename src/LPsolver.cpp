#include <string> 

#include "LPsolver.hpp"

// Implementation of the simplex algorithm: 
// https://en.wikipedia.org/wiki/Simplex_algorithm

// TODO: maybe need to create own matrix class.

// Initialise
LPsolver::LPsolver(std::string * goal_in,
                   std::vector<std::string> * constraints_in) {

    inputGoal = *goal_in;
    inputConstraints = *constraints_in;

    parse();
    standardise();
    createSimplexTableau();
}

void LPsolver::parse() {
    parseInputGoal();

    parseInputConstraints();
}

void parseInputGoal() {
    // Parse the input goal. For example, for the input "1x + -3y",
    // it would parse to {(1, "x"), (-3, "y")}
    int i = 0;
    while (i < inputGoal.length()) {
        char curr = inputGoal[i];

        // Skip things that aren't terms
        if (curr == ' ' || curr == '+') {
            i++;
        }
        
        // TODO: make sure that this elif works, maybe need a continue in the if branch
        // Now we know a term is appearing next
        else if (curr != '+') {
            int coeff;
            std::string::size_type coeffLength;
            // Get the coefficient as an int
            // If this fails, then the coefficient is just a negative sign, 
            // or 1 (no coefficient)
            try {
                coeff = std::stoi(inputGoal.substr(i), &coeffLength);
            } catch (std::invalid_argument) { // TODO: this worked when testing in main
                coeffLength = 1;

                if (i + 1 < inputGoal.length() && inputGoal[i + 1] == '-') {
                    coeff = -1;
                }

                else {
                    coeff = 1;
                }
            }
            
            // Begin reading the variable to which the coefficient is attached
            // First skip over the number
            i += coeffLength;
            std::string var;

            curr = inputGoal[i];
            while (i < inputGoal.length() && inputGoal[i] != ' ') {
                var.push_back(curr);
                i++;
            }

            // If no var name was read, then the coefficient is just a number
            if (var.length() == 0) {
                var = "NUM";
            }

            // Finally save the parsed info
            auto term = std::make_pair(coeff, var);
            parsedGoal.push_back(term);

            variables.insert(var);
            // TODO: maybe need to increment i after this
        }
    }
}

void parseInputConstraints() {
    // Return tuple of list of terms in LHS, comparator, RHS
    // For example, if 5x - -11y <= z is input, then
    // the following tuple is returned: ({(5, "x"), (-11, "y"), (-1, "z")}, "<=", "0")
    std::vector<std::pair<int, std::string>> LHS;
    std::string comparator;
    std::vector<std::pair<int, std::string>>  RHS;

    bool begunRHS = false;
    for (int i = 0; i < inputConstraints.size(); i++) {
        std::string currConstraint = inputConstraints[i];

        int j = 0;
        bool nextTermNegative = false;
        while (j < currConstraint.length()) {
            char curr = currConstraint[j];

            // Skip spaces and additions (since addition is the assumption)
            if (curr == ' ' || curr == '+') {
                j++;
            }

            // Check if next term should be negative
            else if (curr == '-'){
                nextTermNegative = true;
                j++;
            }

            // Check if we're at a comparator
            else if (curr == '=') {
                comparator = "=";
                begunRHS = true;
                j++;
            }

            else if (curr == '<') {
                comparator = "<";
                begunRHS = true;
                j++;
            }

            else if (curr == '>') {
                comparator = ">";
                begunRHS = true;
                j++;
            }

            // Now we are at a term
            else {
                // As for parsing the goal, read the coefficient and variable
                int coeff;
                std::string::size_type coeffLength;

                try {
                    coeff = std::stoi(currConstraint.substr(j), &coeffLength);
                } catch (std::invalid_argument) { // TODO: this worked when testing in main
                    coeffLength = 1;

                    if (j + 1 < currConstraint.length() && currConstraint[j + 1] == '-') {
                        coeff = -1;
                    }

                    else {
                        coeff = 1;
                    }
                }

                if (nextTermNegative) {
                    coeff = -1 * coeff;
                    nextTermNegative = false;
                }

                // Parse the variable name
                j += coeffLength;
                std::string var;

                curr = currConstraint[j];
                while (j < currConstraint.length() && currConstraint[j] != ' ') {
                    var.push_back(curr);
                    j++;
                }

                if (var.length() == 0) {
                    var = "NUM";
                }

                // Finally save the parsed info
                auto term = std::make_pair(coeff, var);

                if (begunRHS) {
                    // Move the term to the LHS unless it's a NUM
                    if (term.second == "NUM") {
                        RHS.push_back(term);
                    }

                    else {
                        term.first = -1 * coeff;
                        LHS.push_back(term);
                    }
                }

                else {
                    LHS.push_back(term);
                }

                variables.insert(var);
                // TODO: maybe need to increment i after this
            }
        }
    }




    // Might have to manipulate to add variables to the RHS
    // Check that no new variables are introduced here

}

void LPsolver::standardise() {
    // Put the goal and constraints into standard form.

    // Eliminate constraints of the form ... >= k, k =/= 0 
    // by substitution with new variables.
    step1();

    // Now all inequality constraints are just positivity constraints.
    // Next, change all inequality constraints to equality constraints
    // using "slack variables".
    step2();

    // Remove unconstrained variables. These can be trivially solved by 
    // the solver (set them to any number that satisfies its constraints).
    // Do this by replacing such constraints with two two equality constraints.
    step3();
}


