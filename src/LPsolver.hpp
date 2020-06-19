#pragma once

#include <vector>
#include <set>
#include <string>

class LPsolver {
    std::string inputGoal;
    std::vector<std::string> inputConstraints;

    std::set<std::string> variables; // TODO: do I even need this?

    std::vector<std::pair<int, std::string>> parsedGoal;
    std::vector
    <
        std::tuple
            <
            std::vector<std::pair<int, std::string>>,
            std::string,
            std::vector<std::pair<int, std::string>>
            >
    > parsedConstraints;
    
    
    // standardised max goal
    // standardised constraints

    void parse();
    void standardise();
    void createSimplexTableau();

    public:
        LPsolver (std::string*, std::vector<std::string>*);

        std::vector<std::string> solve();
} LPsolver;

void parseInputGoal();
void parseInputConstraints();
void step1();
void step2();
void step3();
