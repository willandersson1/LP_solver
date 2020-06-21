#pragma once

#include <vector>
#include <set>
#include <tuple>
#include <string>
#include <Eigen/Dense>

class LPsolver {
    std::string inputGoal;
    std::vector<std::string> inputConstraints;

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

    std::set<std::string> variables;

    Eigen::MatrixXd A;
    Eigen::VectorXd c;
    Eigen::VectorXd b;
    Eigen::MatrixXd simplexTableau;

    void parse();
    void standardise();
    void createSimplexTableau();

    void parseInputGoal();
    void parseInputConstraints();
    void step1();
    void step2();
    void step3();
    void makeAllRHSPositive();

    public:
        LPsolver (std::string, std::vector<std::string>);

        std::vector<std::string> solve();
} LPsolver;