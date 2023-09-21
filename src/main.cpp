#include <iostream>
#include "headers/functionality.h"

int main() {
    printf("Please select what you'd like to do:\n");
    printf("0 - solve a general linear program\n");
    printf("1 - run benchmarks for randomised inputs\n");

    int choice;
    std::cin >> choice;

    switch (choice) {
        case 0: solveGeneralLP(); break;
        case 1: benchmark(); break;
        default: printf("Invalid selection");
    }
}