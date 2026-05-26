pub mod fk_solver;
pub mod ik_solver;
pub mod jacobian_ik_solver;

pub enum IkSolverMethod {
    AnalyticalMethod,
    NumericalMethod,
}
