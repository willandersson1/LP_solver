#pragma once

#include <vector>
#include <set>
#include <tuple>
#include <string>
#include <Eigen/Dense>

// Term
typedef struct Term {
    double coeff; 
    std::string var;
} Term;

inline std::ostream &operator<<(std::ostream &os, Term const &m) {
    return os << m.coeff << m.var;
}


// Goal
typedef std::vector<Term> Goal;

inline std::ostream &operator<<(std::ostream &os, Goal const &m) {
    for (int i = 0; i < m.size() - 1; i++) {
        os << m[i] << " + ";
    }
    os << m[m.size() - 1];
    return os;
}


// Constraint
typedef struct Constraint {
    std::vector<Term> LHS;
    std::string comparator;
    double RHS;
} Constraint;

inline std::ostream &operator<<(std::ostream &os, Constraint const &m) {
    for (int i = 0; i < m.LHS.size() - 1; i++) {
        os << m.LHS[i] << " + ";
    }
    os << m.LHS[m.LHS.size() - 1] << " " << m.comparator << " " << m.RHS;
    return os;
}


class LPsolver {
    std::string inputGoal;
    std::vector<std::string> inputConstraints;

    Goal parsedGoal;
    std::vector<Constraint> parsedConstraints;

    std::vector<Constraint> standardisedConstraints;

    std::set<std::string> variables;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    Eigen::VectorXd c;
    Eigen::MatrixXd simplexTableau; // TODO do I really need A, b, c? 

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
};