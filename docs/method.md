# SPCIES -- Help page for topic 'method'

The following optimization methods are used by Spcies.
Please use `spcies('help', 'method_name')` to get more specific and 
in-depth information about a specific 'method_name'.

Optimization methods:

- 'ADMM': Alternating Direction Method of Multipliers
- 'EADMM': Extended ADMM
- 'FISTA': Fast Iterative Shrinking-Threshold Algorithm

Each MPC formulation may only support a subset of the above methods.
Please refer to the documentation of the MPC formulation using
`spcies('help', 'formulation_name')` for the list of supported methods.
Use `spcies('help', 'MPC')` for the list of available MPC formulations.
