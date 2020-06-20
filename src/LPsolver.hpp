#pragma once

#include <vector>
#include <set>
#include <tuple>
#include <string>

class LPsolver {
    std::string inputGoal;
    std::vector<std::string> inputConstraints;

    std::set<std::string> variables; // TODO: do I even need this? Didn't use in the steps

    std::vector<std::pair<int, std::string>> parsedGoal;
    std::vector
    <
        std::tuple
            <
            std::vector<std::pair<int, std::string>>, // LHS
            std::string, // comparator
            int // RHS
            >
    > parsedConstraints;

    std::vector
    <
        std::tuple
            <
            std::vector<std::pair<double, std::string>>, // LHS
            std::string, // comparator
            double // RHS. Need doubles as might need to divide in step 1.
            >
    > standardisedConstraints;
    
    
    // standardised max goal
    // standardised constraints

    void parse();
    void standardise();
    void createSimplexTableau();

    void parseInputGoal();
    void parseInputConstraints();
    void step1();
    void step2();
    void step3();

    public:
        LPsolver (std::string*, std::vector<std::string>*);

        std::vector<std::string> solve();
} LPsolver;


