# SPCIES -- Help page for topic 'timing'

One of the outputs of the SPCIES solver functions is a structure that contains additional
information of its execution and output. Among this information the user can find the
computation times of its execution. This information is only available if the toolbox 
option 'time' was set to true when generating the solver. Please refer to the help page
on 'options' using `spcies('help', 'options')` for information on the toolbox options.

The returned computation times are:

- 'update_time': Computation time of the operations required to update and initialize
                 the solver. This includes preprocessing of the inputs and any computation
                 of ingredients that need to be recomputed online.
- 'solve_time':  Computation time used by the optimization method itself, i.e., the time
                 spent executing the iterations of the optimization method. 
- 'polish_time': Computation time used for final operations required to prepare the outputs
                 of the function once the iterations of the optimization method have finished.
- 'run_time':    Total computation time of the solver.
                 `run_time = update_time + solve_time + polish_time`.
                 In Matlab MEX the run_time excludes the computation time required for the 
                 MEX-specific operations, i.e., only the computation time of the solver
                 function is measured.
