#include "LPsolver.hpp"
// TODO: should includes from header also go here?

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

            // TODO: maybe need to increment i after this
        }
    }
}

void LPsolver::parseInputConstraints() {
    // Return tuple of list of terms in LHS, comparator, RHS
    // For example, if 5x + -11y <= z is input, then
    // the following tuple is returned: ({(5, "x"), (-11, "y"), (-1, "z")}, "<=", "0").

    bool begunRHS = false;
    for (int i = 0; i < inputConstraints.size(); i++) {
        std::string currConstraint = inputConstraints[i];

        std::vector<std::pair<int, std::string>> LHS;
        std::string comparator;
        int RHS = 0;

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
                        RHS += term.first;
                    }

                    else {
                        term.first = -1 * coeff;
                        LHS.push_back(term);
                    }
                }

                else {
                    LHS.push_back(term);
                }

                
                // TODO: maybe need to increment i after this
            }
            
        }

        auto parsed = std::make_tuple(LHS, comparator, RHS);
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
    // Do this by replacing such constraints with two two equality constraints.
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
        auto currComparator = std::get<1>(currTuple);
        auto currRHS = std::get<2>(currTuple);

        // Check if it involves only one variable, since
        // after parsing, all non-NUM variables are in the LHS
        // Also make sure it's a lower bound
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
        // TODO: check if need pointers here. [], get apparently give references
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
            slackName.append(std::to_string(nextSlackVarIndex));
            nextSlackVarIndex++;

            // Update LHS, change comparator to = since we've added slack
            auto slackVar = std::make_pair(coeff, slackName);
            currLHS.push_back(slackVar);

            currComparator = "=";
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
    // This also gives us an order to the variables, due to the set class.
    std::set<std::string> variables;

    // Take the ones from the goal first. By assumption these have no "NUM" vars.
    for (int i = 0; i < parsedGoal.size(); i++) {
        std::string currVar = parsedGoal[i].second;
        variables.insert(currVar);
    }

    // Take the rest from the constraints.
    for (int i = 0; i < standardisedConstraints.size(); i++) {
        auto currTuple = standardisedConstraints[i];
        auto currLHS = std::get<0>(currTuple);

        for (int j = 0; j < currLHS.size(); j++) {
            std::string currVar = currLHS[j].second;

            if (currVar != "NUM") {
                variables.insert(currVar);
            }
        }
    }

    // Create c:
    // If we want to minimise 3x + 2y + -5z, c is [3 2 -5] transposed.
    // If we introduced slack variables, then they need to be appended
    // as zeroes to c. So if we have s_1 and s_2, c becomes [3 2 -5 0 0] transposed.
    // In the end, since we assume inputted constraints don't introduce variables not in 
    // the inputted goal, we can just set the size of c to the number of variables.
    c.resize(variables.size());
    c.setZero();
    
    // Put the goal variables in first, the rest (slack vars) are zero.
    for (int i = 0; i < parsedGoal.size(); i++) {
        c(i) = parsedGoal[i].first;
    }

    // Create A and b...


    // Create the appropiate 0 vector(s)...

    // Make the whole tableau...



}