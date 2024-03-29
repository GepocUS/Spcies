# SPCIES -- Help page for topic 'laxMPC'

laxMPC is a standard MPC formulation without terminal constraint.
This MPC formulation is given in equation (9) of the article:

> Krupa, P., Limon, D., & Alamo, T. (2020). Implementation of model predictive
control in programmable logic controllers. IEEE Transactions on Control Systems
Technology, 29(3), 1117-1130.

The formulation is given by:

```
    min  ||x_N - x_r||^2_T + Sum{i=0:N-1} ||x_i - x_r||^2_Q + ||u_i - u_r||^2_R
    s.t. x_0 = x(k)
         x_{i+1} = A x_i + B u_i, i=0:N-1
         LBu <= u_i <= UBu, i=0:N-1
         LBx <= x_i <= UBx, i=1:N-1
```

Fields of the 'sys' structure:

- Basic 'sys' fields A, B, LBx, UBx, LBu, UBu; see `spcies('help', 'sys')`

Fields of the 'param' structure:

- Basic 'param' fields N, Q, R, T; see `spcies('help', 'param')`

Available methods, submethods and solver_options:

- 'ADMM'
    - No submethods
    - 'solver_options': Defaults for 'ADMM', see `spcies('help', 'ADMM')`
    - Restrictions: Q and R must be diagonal positive semidefinite.
- 'FISTA'
    - No submethods
    - 'solver_options': Defaults for 'FISTA', see `spcies('help', 'FISTA')`
    - Restrictions: Q, R and T must be diagonal positive definite.
