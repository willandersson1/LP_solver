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
    std::cout << "Parsed goal:\n" << parsedGoal << std::endl;
}

void LPsolver::parseInputConstraints() {
    for (const std::string& currConstraint : inputConstraints) {
        std::cout << "curr cstr " << currConstraint << std::endl;
        assert(currConstraint.find(">") == std::string::npos); // don't allow >= for now

        Constraint parsedCstr;

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

                parsedCstr.comparator = "<=";
                parsedCstr.RHS = std::stoi(currConstraint.substr(j), nullptr, 10);
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
    std::cout << "Parsed constraints:" << std::endl;
    for (Constraint c : parsedConstraints) {
        std::cout << c << std::endl;
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
    for (const Constraint& currCstr : parsedConstraints) {
        std::vector<Term> currLHS = currCstr.LHS;
        std::string currComparator = currCstr.comparator;
        double currRHS = currCstr.RHS;

        // Check if it involves only one variable, since
        // after parsing, all non-NUM variables are in the LHS
        // Also make sure it's a lower bound
        // TODO: what is this for
        if (currLHS.size() == 1 && currComparator == ">=") {
            double oldCoeff = currLHS[0].coeff;
            std::string oldVar = currLHS[0].var;

            // Introduce a new variable
            // Move RHS to LRS (so looks like ... >= 0),
            // also turn coefficient of newVar into 1.
            std::string newVar = oldVar.append("_fresh");
            double constTerm = -1 * (currRHS / oldCoeff);

            // Mark it for substitution
            freshExpressions.push_back(std::make_tuple(oldVar, newVar, constTerm));

            // Note: by not adding it to the new constraints list, 
            // it effectively deletes this constraint.
        }

        // It doesn't involve one variable, so we don't manipulate it
        else {
            standardisedConstraints.push_back(currCstr);
        }
    }
    
    // Now substitute with the newly minted variables
    // Iterate over all constraints without single variables
    // TODO I think I'm messed up with the pointers etc here
    for (Constraint& currCstr : standardisedConstraints) {
        std::cout << "before cstr: " << currCstr << std::endl;
        // Check all pairs in the LHS to see if any need substitution
        // Iterate over all coeff/variable pairs in the LHS
        // for (int j = 0 ; j < currLHS.size(); j++) { // TODO turn into for Term : currL..
        for (Term& currTerm : currCstr.LHS) {            
            // Look for matches in all variables that should be substituted
            for (int k = 0; k < freshExpressions.size(); k++) {
                std::string oldVar = std::get<0>(freshExpressions[k]);
                std::string newVar = std::get<1>(freshExpressions[k]);
                double newConst = std::get<2>(freshExpressions[k]);

                // Found match: substitute
                if (oldVar == currTerm.var) {
                    currTerm.var = newVar;
                    // Multiply the const in the expression
                    // by the current coefficient, then move it to the RHS
                    currCstr.RHS -= currTerm.coeff * newConst;
                }
            }
        }
    std::cout << "after cstr: " << currCstr << "\n" << std::endl;
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
    for (Constraint& currCstr : standardisedConstraints) {
        // Update all non-equality constraints
        if (currCstr.comparator != "=") {
            int coeff = 1;

            if (currCstr.comparator == ">=") {
                coeff = -1;
            }

            std::string slackName = "s_" + nextSlackVarIndex;
            nextSlackVarIndex++;

            // Update LHS, change comparator to = since we've added slack
            auto slackVar = std::make_pair(coeff, slackName);
            currCstr.LHS.push_back({(double) coeff, slackName});
            currCstr.comparator = "=";
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
    std::vector<std::string> variables; // TODO not a set? to do with order?

    // Take the ones from the goal first. By assumption these have no "NUM" vars.
    for (Term term : parsedGoal) {
        std::string currVar = term.var;
        variables.push_back(currVar);
    }

    // Take the rest from the constraints.
    // for (int i = 0; i < standardisedConstraints.size(); i++) { // TODO do for cstr : st ..
    //     Constraint currCstr = standardisedConstraints[i];
    for (const Constraint& currCstr : standardisedConstraints) {
        for (const Term& term : currCstr.LHS) {
            std::string currVar = term.var;

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

        const Constraint currCstr = standardisedConstraints[i];
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
    // TODO check the ordering
    Eigen::RowVectorXd goalVec(variables.size());
    for (int i = 0; i < parsedGoal.size(); i++) { // do for .. : ..
        goalVec(i) = parsedGoal[i].coeff;
    }
    simplexTableau.row(standardisedConstraints.size()) << -goalVec, 1, 0;
    
    std::cout << "Simplex tableu:\n" << simplexTableau << std::endl;
}