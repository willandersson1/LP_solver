use std::error::Error;
use good_lp::{constraint, default_solver, Solution, SolverModel, variables};
#[derive(Debug, PartialEq)]
struct PolynomialTerm {
    coefficient: i32,
    variable: char,
}
impl From<&str> for PolynomialTerm {
    fn from(arg: &str) -> Self {
        //either no coefficient (so 1) or parse everything but the bariable
        let coefficient: i32 = match arg.len() {
            1 => 1,
            _ => arg[0..arg.len() - 1].parse().unwrap(),
        };

        let variable = arg.chars().last().unwrap();
        assert!(variable.is_alphabetic());
        PolynomialTerm {
            coefficient,
            variable,
        }
    }
}

#[derive(Debug, PartialEq)]
struct Polynomial(Vec<PolynomialTerm>);

impl From<&str> for Polynomial {
    fn from(arg: &str) -> Self {
        let mut last_was_sign = false;
        let mut last_was_neg = false;
        let mut polynomial: Vec<PolynomialTerm> = vec![];
        arg.split_whitespace().for_each(|token| {
            if ["+", "-"].contains(&token) {
                if last_was_sign {
                    panic!("Encountered two signs in a row in {}", arg)
                } else {
                    last_was_sign = true
                }
            }
            match token {
                "+" => last_was_neg = false,
                "-" => last_was_neg = true,
                " " => {}
                term => {
                    last_was_sign = false;
                    let mut term: PolynomialTerm = term.into();
                    if last_was_neg {
                        term.coefficient *= -1;
                        last_was_neg = false;
                    }
                    polynomial.push(term);
                }
            }
        });
        return Polynomial(polynomial);
    }
}

struct Equation {
    lhs: Polynomial,
    rhs: Polynomial,
}

fn main() -> Result<(), Box<dyn Error>> {

    //x, y >= 0
    variables! {
        vars:
               x >= 0;
               y >= 0;
    } 

    //2x - 3y
    let solution = vars.minimise(2 * x - 3 * y)
        .using(default_solver) // multiple solvers available
        //3x + 2y <= 10
        .with(constraint!(3 * x + 2 * y <= 10))
        //2x + 5y <= 15
        .with(constraint!(2 * x + 5 * y <= 15))
        .solve()?;
    println!("a={}   b={}", solution.value(x), solution.value(y));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polynomial_term_parse() {
        let term: PolynomialTerm = "5x".into();
        assert_eq!(
            term,
            PolynomialTerm {
                coefficient: 5,
                variable: 'x'
            }
        );

        let term: PolynomialTerm = "y".into();
        assert_eq!(
            term,
            PolynomialTerm {
                coefficient: 1,
                variable: 'y'
            }
        );
    }

    #[test]
    fn test_polynomial_parse() {
        let polynomial: Polynomial = "2x + 3y - 5z".into();
        assert_eq!(
            polynomial,
            Polynomial(vec![
                PolynomialTerm {
                    coefficient: 2,
                    variable: 'x'
                },
                PolynomialTerm {
                    coefficient: 3,
                    variable: 'y'
                },
                PolynomialTerm {
                    coefficient: -5,
                    variable: 'z'
                }
            ])
        );
    }
}
