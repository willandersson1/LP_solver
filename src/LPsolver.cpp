#include "headers/LPsolver.h"
#include <iostream>
#include <vector>

// Implementation of the simplex algorithm: 
// https://en.wikipedia.org/wiki/Simplex_algorithm
// https://math.libretexts.org/Bookshelves/Applied_Mathematics/Applied_Finite_Mathematics_(Sekhon_and_Bloom)/04%3A_Linear_Programming_The_Simplex_Method/4.02%3A_Maximization_By_The_Simplex_Method
// https://www.math.wsu.edu/students/odykhovychnyi/M201-04/Ch06_1-2_Simplex_Method.pdf 

// Helper
Term getTermFromSubstring(std::string in_str, int* idx) {
    // stoi will write the length of the number here
    std::string::size_type coeffLength;
    // grabs the first integer in the remaining part of the string
    int coeff = std::stoi(in_str.substr(*idx), &coeffLength, 10);

    // Begin reading the variable to which the coefficient is attached
    // First skip over the number
    *idx += coeffLength;
    std::string var;
    while (*idx < in_str.length()) {
        char curr = in_str[*idx];

        if (curr == ' ') {
            break;
        }

        var.push_back(curr);
        *idx += 1;
    }

    return {coeff, var};
}


LPsolver::LPsolver(std::string goal_in,
                   std::vector<std::string> constraints_in) {
    inputGoal = goal_in;
    inputConstraints = constraints_in;

    parse();
    createSimplexTableau();
}

int getPivotCol(Eigen::MatrixXd* simplexTableau) {
    int rows = simplexTableau -> rows();
    int cols = simplexTableau -> cols();

    int pivotCol = 0;
    double pivotColVal = (*simplexTableau)(rows - 1, pivotCol);
    for (int c = 1; c < cols - 2; c++) { // subtract 2 because we skip the b and goal columns
        double elem = (*simplexTableau)(rows - 1, c);
        if (elem < pivotColVal) {
            pivotCol = c;
            pivotColVal = elem;
        }
    }

    return pivotCol;
}

int getPivotRow(Eigen::MatrixXd* simplexTableau, int pivotCol) {
    int rows = simplexTableau -> rows();
    int cols = simplexTableau -> cols();

    std::vector<int> positivePivotColIdxs;
    for (int r = 0; r < rows - 1; r++) {
        if ((*simplexTableau)(r, pivotCol) > 0) {
            positivePivotColIdxs.push_back(r);
        }
    }

    int pivotRow = -1;
    double smallestQuotient;
    for (int r : positivePivotColIdxs) {
        double quotient = (*simplexTableau)(r, cols - 1) / (*simplexTableau)(r, pivotCol);
        if (pivotRow == -1 || quotient < smallestQuotient) {
            pivotRow = r;
            smallestQuotient = quotient;
        }
    }

    return pivotRow;
}

std::string LPsolver::getResults() {
    const int rows = simplexTableau.rows();
    const int cols = simplexTableau.cols();

    std::string output = "Results:\n";
    // Ignore slack/extra variables
    for (int i = 0; i < parsedGoal.size(); i++) {
        int numPos = 0;
        int lastPos_idx = -1;
        for (int r = 0; r < rows; r++) {
            if (simplexTableau(r, i) > 0) {
                lastPos_idx = r;
                numPos++;
            }
        }

        assert(numPos > 0);
        // TODO why is this messed up when I have a negative coeff in the goal? I think bc 
        // there's a negative number (from the positive coeff) so it does that, then it's just a positive 
        // number (from the negative coeff) left. With "999x + -1y", {"5x + -1y <= 9", "1x <= 2"}
        // it's fine, but with "5x + -2y" it's not.
        double varCoeff = numPos == 1 ? simplexTableau(lastPos_idx, cols - 1) : 0;
        output += variables[i] + ": " + std::to_string(varCoeff) + "\n";
    }
    output += "Optimal value: " + std::to_string(simplexTableau(rows - 1, cols - 1));
    
    return output;
}

void LPsolver::solve() {
    const int rows = simplexTableau.rows();
    const int cols = simplexTableau.cols();

    while (simplexTableau.row(rows - 1).minCoeff() < 0) {
        int pivotCol = getPivotCol(&simplexTableau);
        int pivotRow = getPivotRow(&simplexTableau, pivotCol);
        double pivotElem = simplexTableau(pivotRow, pivotCol);
        
        // Apparently these always hold
        assert(pivotRow < rows - 1);
        assert(pivotElem > 0);

        // Pivot
        // Make pivot = 1
        simplexTableau.row(pivotRow) /= pivotElem;

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
    std::cout << "Final tableau:\n" << simplexTableau << std::endl;
}

void LPsolver::parse() {
    parseInputGoal();
    parseInputConstraints();
}

void LPsolver::parseInputGoal() {
    int i = 0;
    while (i < inputGoal.length()) {
        char curr = inputGoal[i];

        // Skip things that aren't terms
        if (curr == ' ' || curr == '+') {
            i++;
            continue;
        }
        
        Term term = getTermFromSubstring(inputGoal, &i);
        parsedGoal.push_back(term);
    }
}

void LPsolver::parseInputConstraints() {
    for (const std::string& currConstraint : inputConstraints) {
        Constraint parsedCstr;

        int j = 0;
        while (j < currConstraint.length()) {
            char curr = currConstraint[j];

            // Skip spaces and additions (since addition is the assumption)
            if (curr == ' ' || curr == '+') {
                j++;
            }

            // Check if we're at a comparator
            else if (curr == '<' || curr == '>') {
                parsedCstr.comparator += curr; parsedCstr.comparator += currConstraint[j + 1];                
                j += 2;
                parsedCstr.RHS = std::stoi(currConstraint.substr(j), nullptr, 10);

                assert(parsedCstr.comparator == "<=" || parsedCstr.comparator == ">=");
                assert(parsedCstr.RHS >= 0);

                break;
            }

            // Now we are at a term, in the LHS
            else {
                Term term = getTermFromSubstring(currConstraint, &j);
                parsedCstr.LHS.push_back(term);
            }
        }
        parsedConstraints.push_back(parsedCstr);
    }
    for (auto c : parsedConstraints) std::cout << c << std::endl;
}

void LPsolver::addSlackVariables() {
    // Now we want to get rid of all inequality constraints and replace them 
    // with equality constraints. This allows for simpler solving.
    // Introduce "slack variables" that are >= 0 (again, implicitly).
    // Example: x + 2y <= 2 becomes x + 2y + s = 2, with s >= 0 a slack variable.
    // The intuition is that x <= 0 iff there's *some* s >= 0 s.t
    // x + s = 0. It "tightens the slack" to "bring up" x to 0.
    // The same but with bringing it down for >= (ie y >= 0 ~> y - s' = 0)
    // standardisedConstraints = parsedConstraints;
    int nextSlackVarIndex = 0;
    for (Constraint& currCstr : parsedConstraints) {
        // Update all non-equality constraints
        if (currCstr.comparator != "=") {
            int coeff = currCstr.comparator == "<=" ? 1 : -1;

            std::string slackName = "s_" + std::to_string(nextSlackVarIndex);
            for (std::string s : variables) assert(slackName != s); // make sure not already a variable
            nextSlackVarIndex++;

            // Update LHS, change comparator to = since we've added slack
            currCstr.LHS.push_back({(double) coeff, slackName});
            currCstr.comparator = "=";
        }
    }
    std::cout << "\nFinal constraints" << std::endl;
    for (auto c : parsedConstraints) std::cout << c << std::endl;
}

void LPsolver::createSimplexTableau() {
    // First step, to get a canonical form
    addSlackVariables();

    // Gather list of all variables
    for (Term term : parsedGoal) {
        variables.push_back(term.var);
    }

    // Take the rest from the constraints.
    for (const Constraint& currCstr : parsedConstraints) {
        for (const Term& term : currCstr.LHS) {
            std::string currVar = term.var;

            // Avoid "NUM" and duplicates.
            if (currVar != "NUM" && 
                std::count(variables.begin(), variables.end(), currVar) == 0) {
                variables.push_back(currVar);
            }
        }
    }

    // Begin actually making the simplex tableau
    // One row per constraint.
    // Have the form [LHS, 0, RHS] where RHS is a constant and LHS
    // is the coefficients of the relevant variables. 
    // Here we need to use the ordering of variables in the set to keep track.
    int rows = 1 + parsedConstraints.size();
    int cols = variables.size() + 2; // one for b, one for the bottom-most 1 for goal
    simplexTableau.resize(rows, cols);

    // Iterate over all the constraints and put them into the matrix.
    for (int i = 0; i < parsedConstraints.size(); i++) {
        Eigen::RowVectorXd currConstraintCoeffs(variables.size());
        currConstraintCoeffs.setZero();

        const Constraint currCstr = parsedConstraints[i];
        double currRHS = currCstr.RHS;

        // Go through the LHS of this constraint to get the coefficients.
        for (Term term : currCstr.LHS) {
            double coeff = term.coeff;
            std::string var = term.var;
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
    Eigen::RowVectorXd goalVec(variables.size());
    goalVec.setZero();
    for (int i = 0; i < parsedGoal.size(); i++) {
        goalVec(i) = parsedGoal[i].coeff;
    }
    simplexTableau.row(rows - 1) << -goalVec, 1, 0;
    
    std::cout << "Simplex tableu:\n" << simplexTableau << std::endl;
}