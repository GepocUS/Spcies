# SPCIES -- Help page for topic 'param'

The 'param' structure is one the two basic structures used to 
define the MPC solver. It contains the ingredients of the MPC.
Each MPC formulation has its own ingredients, but they all share
the same names and have approximately the same meaning.

Possible fields of 'param' are the following (values in parentheses
depend on the MPC and optimization method):

- 'N': Prediction horizon. Integer > 2.
- 'Q': Cost function matrix for the state of the system.
       Assumed to be positive (diagonal) (semi)definite.
- 'R': Cost function matrix for the input of the system.
       Assumed to be positive (diagonal) (semi)definite.
- 'T': Terminal cost function matrix for the terminal state or
       artificial reference for the state.
       Assumed to be positive (semi)definite.
- 'S': Terminal cost function matrix for the artificial reference
       for the input.
       Assumed to be positive (semi)definite.
