#include "headers/LPsolver.h"
#include <iostream>
#include <vector>

// Implementation of the simplex algorithm: 
// https://en.wikipedia.org/wiki/Simplex_algorithm

// Initialise
LPsolver::LPsolver(std::string goal_in,
                   std::vector<std::string> constraints_in) {
    inputGoal = goal_in;
    inputConstraints = constraints_in;

    parse();
    standardise();
    createSimplexTableau();
}

std::vector<std::string> LPsolver::solve() {
    const int rows = simplexTableau.rows();
    const int cols = simplexTableau.cols();

    while (simplexTableau.row(rows - 1).minCoeff() < 0) {
        // Get pivot col
        int pivotCol = 0;
        double pivotColVal = simplexTableau(rows - 1, 0);
        for (int c = 1; c < cols - 2; c++) { // subtract 2 because we skip the P value and the b column
            double elem = simplexTableau(rows - 1, c);
            if (elem < pivotCol) {
                pivotCol = c;
                pivotColVal = elem;
            }
        }

        // Get pivot row
        Eigen::VectorXd quotients(rows - 1);
        quotients.setZero();
        std::vector<int> positivePivotColIdxs;
        for (int r = 0; r < rows - 1; r++) {
            if (simplexTableau(r, pivotCol) > 0) {
                positivePivotColIdxs.push_back(r);
            }
        }

        int pivotRow = -1;
        double smallestQuotient;
        for (int r : positivePivotColIdxs) {
            double quotient = simplexTableau(r, cols - 1) / simplexTableau(r, cols - 1);
            if (pivotRow == -1 || quotient < smallestQuotient) {
                pivotRow = r;
                smallestQuotient = quotient;
            }
        }

        double pivotElem = simplexTableau(pivotRow, pivotCol);
        assert(pivotRow < rows - 1);
        assert(pivotElem > 0);

        // Pivot
        // Make pivot = 1
        simplexTableau.row(pivotRow) /= pivotElem;
        std::cout << "after dividing by pivot elem\n" << simplexTableau << std::endl;

        // Make all other entries in pivot col = 0
        for (int r = 0; r < rows; r++) {
            if (r == pivotRow) {
                continue;
            }
            
            double elem = simplexTableau(r, pivotCol);
            simplexTableau.row(r) -= elem * simplexTableau.row(pivotRow);
        }
        std::cout << "after pivoting\n" << simplexTableau << std::endl;
    }

    std::cout << "final\n" << simplexTableau << std::endl;
    
    std::vector<std::string> v = {"solver"};
    return v;
}

void LPsolver::parse() {
    parseInputGoal();
    parseInputConstraints();
}

void LPsolver::parseInputGoal() {
    // Parse the input goal. For example, for the input "1x + -3y",
    // it would parse to {(1, "x"), (-3, "y")}.
    int i = 0;
    while (i < inputGoal.length()) {
        char curr = inputGoal[i];

        // Skip things that aren't terms
        if (curr == ' ' || curr == '+') {
            i++;
            continue;
        }
        
        // TODO: factor this part, I use the same code for constraints
        std::string::size_type coeffLength; // stoi will write the length of the number here
        int coeff = std::stoi(inputGoal.substr(i), &coeffLength, 10); // grabs the first integer in the remaining part of the string
        
        // Begin reading the variable to which the coefficient is attached
        // First skip over the number
        i += coeffLength;
        std::string var;

        while (i < inputGoal.length()) {
            curr = inputGoal[i];

            if (curr == ' ') {
                break;
            }

            var.push_back(curr);
            i++;
        }

        // Finally save the parsed info
        auto term = std::make_pair(coeff, var);
        parsedGoal.push_back(term);
    }
}

void LPsolver::parseInputConstraints() {
    // Return tuple of list of terms in LHS, RHS.
    // For example, if 5x + -11y + 1z <= 5 is input, then
    // the following tuple is returned: ({(5, "x"), (-11, "y"), (1, "z")}, "5").
    for (std::string currConstraint : inputConstraints) {
        std::cout << "curr cstr " << currConstraint << std::endl;
        std::vector<std::pair<int, std::string>> LHS;
        int RHS = 0;

        int j = 0;
        while (j < currConstraint.length()) {
            char curr = currConstraint[j];

            // Skip spaces and additions (since addition is the assumption)
            if (curr == ' ' || curr == '+') {
                j++;
            }

            // Check if we're at a comparator
            else if (curr == '<') {
                assert(currConstraint[j + 1] == '=');
                j += 2;

                RHS = std::stoi(currConstraint.substr(j), nullptr, 10); // grabs the first integer in the remaining part of the string
                break;
            }

            // Now we are at a term, in the LHS
            else {
                // As for parsing the goal, read the coefficient and variable
                // TODO: factor this
                std::string::size_type coeffLength; // stoi will write the length of the number here
                int coeff = std::stoi(currConstraint.substr(j), &coeffLength, 10); // grabs the first integer in the remaining part of the string
                
                // Begin reading the variable to which the coefficient is attached
                // First skip over the number
                j += coeffLength;
                std::string var;

                while (j < currConstraint.length()) {
                    curr = currConstraint[j];

                    if (curr == ' ') {
                        break;
                    }

                    var.push_back(curr);
                    j++;
                }

                // Finally save the parsed info
                auto term = std::make_pair(coeff, var);
                std::cout << "term: " << coeff << ", " << var << std::endl;
                LHS.push_back(term);
            }
        }

        auto parsed = std::make_tuple(LHS, RHS);
        parsedConstraints.push_back(parsed);
    }
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
    // Do this by replacing such constraints with two two-equality constraints.
    // TODO: unimplemented. Just enforced by input instructions 
    step3();

    // The final step is to make the RHSs all positive
    makeAllRHSPositive();
}

void LPsolver::step1() {
    // Idea is to convert x >= 7 into a new variable x_fresh := x - 7
    // Then all instances of x in other constraints can be replaced by 
    // x_fresh, as long as x_fresh >= 0. But since all variables in linear programs
    // satisfy this condition, we can just delete the original constraint (x >= 7).
    // It is not necessary to add another constraint for x_fresh.
    std::vector<std::tuple<std::string, std::string, double>> freshExpressions;

    // Iterate over all constraints and find those which only involve one variable
    for (int i = 0; i < parsedConstraints.size(); i++) {
        auto currTuple = parsedConstraints[i]; 
        auto currLHS = std::get<0>(currTuple);
        auto currComparator = "<="; // TODO: make string
        auto currRHS = std::get<1>(currTuple);

        // Check if it involves only one variable, since
        // after parsing, all non-NUM variables are in the LHS
        // Also make sure it's a lower bound
        // TODO: what is this for
        if (currLHS.size() == 1 && currComparator == ">=") {
            double oldCoeff = currLHS[0].first;
            std::string oldVar = currLHS[0].second;

            // Introduce a new variable
            // Move RHS to LRS (so looks like ... >= 0),
            // also turn coefficient of newVar into 1.
            std::string newVar = oldVar.append("_fresh");
            double constTerm = -1 * (currRHS/oldCoeff);

            // Mark it for substitution
            freshExpressions.push_back(std::make_tuple(oldVar, newVar, constTerm));

            // Note: by not adding it to the new constraints list, 
            // it effectively deletes this constraint.
        }

        // It doesn't involve one variable, so we don't manipulate it
        // Need to cast ints to doubles explicitly
        else {
            std::vector<std::pair<double, std::string>> newLHS;
            for (int j = 0; j < currLHS.size(); j++) {
                newLHS.push_back(std::make_pair((double) currLHS[j].first, currLHS[j].second));
            }

            double newRHS = (double) currRHS;

            standardisedConstraints.push_back(std::make_tuple(newLHS, currComparator, newRHS));
        }
    }
    
    // Now substitute with the newly minted variables
    // Iterate over all constraints without single variables
    for (int i = 0; i < standardisedConstraints.size(); i++) {
        // TODO: test if need pointers here actually. [] and get should give references
        auto currTuple = standardisedConstraints[i]; 
        auto currLHS = std::get<0>(currTuple); 
        double* currRHS = &std::get<2>(currTuple);

        // Check all pairs in the LHS to see if any need substitution
        // Iterate over all coeff/variable pairs in the LHS
        for (int j = 0 ; j < currLHS.size(); j++) {
            double currCoeff = currLHS[j].first;
            std::string* currVar = &currLHS[j].second; // TODO: also this pointer
            
            // Look for matches in all variables that should be substituted
            for (int k = 0; k < freshExpressions.size(); k++) {
                std::string oldVar = std::get<0>(freshExpressions[k]);
                std::string newVar = std::get<1>(freshExpressions[k]);
                double newConst = std::get<2>(freshExpressions[k]);

                // Found match: substitute
                if (oldVar == *currVar) {
                    *currVar = oldVar;
                    
                    // Multiply the const in the expression
                    // by the current coefficient, then move it to the RHS
                    *currRHS -= currCoeff * newConst;
                }
            }
        }
    }
}

void LPsolver::step2() {
    // Now we want to get rid of all inequality constraints and replace them 
    // with equality constraints. This allows for simpler solving.
    // Introduce "slack variables" that are >= 0 (again, implicitly).
    // Example: x + 2y <= 2 becomes x + 2y + s = 2, with s >= 0 a slack variable.
    // The intuition is that x <= 0 iff there's *some* s >= 0 s.t
    // x + s = 0. It "tightens the slack" to "bring up" x to 0.
    // The same but with bringing it down for >= (ie y >= 0 ~> y - s' = 0)

    int nextSlackVarIndex = 0;
    for (int i = 0; i < standardisedConstraints.size(); i++) {
        // TODO: check why the pointers are so weird here
        auto currTuple = standardisedConstraints[i];
        auto currLHS = std::get<0>(currTuple);
        auto currComparator = std::get<1>(currTuple);

        // Update all non-equality constraints
        if (currComparator != "=") {
            int coeff = 1;

            if (currComparator == ">=") {
                coeff = -1;
            }

            std::string slackName = "s_";
            slackName.append(std::to_string(nextSlackVarIndex)); // TODO could just concat at initialisation...
            nextSlackVarIndex++;

            // Update LHS, change comparator to = since we've added slack
            auto slackVar = std::make_pair(coeff, slackName);
            std::get<0>(standardisedConstraints[i]).push_back(slackVar);

            std::get<1>(standardisedConstraints[i]) = "=";
        }
    }
}

void LPsolver::step3() {
    // Final step is to remove unrestricted variables. These are ones
    // that have no constraint limiting them to a domain
    // if x is unrestricted, then introduce the expression x_+ - x_- 
    // with x_+, x_- >= 0. Substituting x by this expression is equivalent
    // to x being unrestricted.
    // Since we implicitly assume all variables remaining after standardising 
    // are >= 0, we don't need to add another constraint and x can be elimintated
    // from the program.
}

void LPsolver::makeAllRHSPositive() {
    // TODO
}

void LPsolver::createSimplexTableau() {
    // Because the program has been standardised, we know that we can
    // create a simplex tableau in canonical form. 
    // See https://en.wikipedia.org/wiki/Simplex_algorithm#Simplex_tableau
    // The linear algebra library Eigen is used.

    // Collect all variables except the artificial "NUM" ones.
    // This also gives us an order to the variables, so we can refer to them.
    std::vector<std::string> variables;

    // Take the ones from the goal first. By assumption these have no "NUM" vars.
    for (int i = 0; i < parsedGoal.size(); i++) {
        std::string currVar = parsedGoal[i].second;
        variables.push_back(currVar);
    }

    // Take the rest from the constraints.
    for (int i = 0; i < standardisedConstraints.size(); i++) {
        auto currTuple = standardisedConstraints[i];
        auto currLHS = std::get<0>(currTuple);

        for (int j = 0; j < currLHS.size(); j++) {
            std::string currVar = currLHS[j].second;

            // Avoid "NUM" and duplicates.
            if (currVar != "NUM" && 
                std::count(variables.begin(), variables.end(), currVar) == 0) {
                variables.push_back(currVar);
            }
        }
    }

    // Create the simplex tableau.

    // One row per constraint.
    // Have the form [LHS, 0, RHS] where RHS is a constant and LHS
    // is the coefficients of the relevant variables. 
    // Here we need to use the ordering of variables in the set to keep track.
    int rows = 1 + standardisedConstraints.size();
    int cols = variables.size() + 2; // one for b, one for the bottom-most 1 for goal
    simplexTableau.resize(rows, cols);

    // Iterate over all the constraints and put them into the matrix.
    for (int i = 0; i < standardisedConstraints.size(); i++) {
        Eigen::RowVectorXd currConstraintCoeffs(variables.size());

        auto currTuple = standardisedConstraints[i];
        double currRHS = std::get<2>(currTuple);
        auto currLHS = std::get<0>(currTuple);

        // Go through the LHS of this constraint to get the coefficients.
        for (int j = 0; j < currLHS.size(); j++) {
            double coeff = std::get<0>(currLHS[j]);
            std::string var = std::get<1>(currLHS[j]);
            int varIdx;

            // Get the column index that represents this var.
            // Add 1 because the 0th column is a 0.
            for (int k = 0; k < variables.size(); k++) {
                if (variables[k] == var) {
                    varIdx = k;
                    break;
                }
            }
            currConstraintCoeffs(varIdx) = coeff;
        }
        simplexTableau.row(i) << currConstraintCoeffs, 0, currRHS;
    }

    // Goal row, at the bottom
    // TODO check the ordering
    Eigen::RowVectorXd goalVec(variables.size());
    for (int i = 0; i < parsedGoal.size(); i++) {
        goalVec(i) = parsedGoal[i].first;
    }
    simplexTableau.row(standardisedConstraints.size()) << -goalVec, 1, 0;

    std::cout << "Simplex tableu:\n" << simplexTableau << std::endl;
}