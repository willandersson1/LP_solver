(work in progress)

### Overview
This program will solve linear programs by using the simplex algorithm, created by George Dantzig in 1947 (https://en.wikipedia.org/wiki/Simplex_algorithm). A linear program is given by an optimisation goal and a list of linear constraints. For example: 

	Goal: minimise 2x - 3y
	With: 3x + 2y <= 10
              2x + 5y <= 15
              x, y >= 0

A linear program solver will then find the values for x and y such that the goal is minimised. Linear programs are a core concept of mathematical optimisation, and have therefore been studied extensively. However, to solve a linear program is still fairly inefficient: state of the art algorithms are still worse than quadratic in the number of variables. The simplex algorithm is generally cubic but can be exponential for some inputs. Some problems are better solved using specified algorithms, such as the max flow problem.


### Motivation
Linear programs weren't mentioned in any of my algorithms classes; I learned of them from a finance textbook. I looked the topic up on Wikipedia and was astounded to see how much research goes into these algorithms. I didn't think for a second that business problems, given by only a few formulas, could be so difficult to solve in general. If asked for a complicated algorithm, a fellow computer science student would never mention linear programs. After all this I decided it would be a good challenge to try and implement one myself, also partly because I could practise parsing user input.


### Explanation of simplex algorithm
...


### Usage
When inputting the program, you must follow this list of rules so that the program can parse your input correctly: 

* ... 




### Installation
* Eigen
* gcc
* cmake (todo)
