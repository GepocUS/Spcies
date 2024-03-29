# SPCIES -- Help page for topic 'options'

The SPCIES toolbox has two categories of options: 'solver_options' and 'options'

'solver_options' are options that are particular to each optimization method.
Please refer to the documentation of each method using `spcies('help', 'method_name')`
for its list of 'solver_options'.
Execute `spcies('help', 'method')` for the list of available methods.

'options' are the general options of the toolbox. The options are:

- 'method': String. Determines the optimization method used by the solver.
            Use `spcies('help', 'method')` for the list of supported methods.
            Each MPC formulation may only supports a subset of the methods.
- 'submethod': String. Determines the submethod of optimization method used by
              the solver. Some MPC formulations may have more than one solver
              that uses the same optimization method. If `submethod == ''`, the
              default 'submethod' is used.
- 'formulation': String. Determines the MPC formulation to be constructed.
                 Use `spcies('help', 'MPC')` for the list of available MPC formulations.
- 'platform': String, either 'C' or 'Matlab'. Determines if the solver is generated
              in plain C or if it is also generated and compiled for Matlab MEX.
- 'save_name': String. Determines the name of the generated files.
- 'directory': String. Path where the generated solver files are saved.
               If unspecified, the toolbox saved all files its subdirectory
               `generated_solvers\`, whose location can be found by executing
               `spcies_get_root_directory()`;
- 'override': Boolean. Determines if files can be overwritten. Defaults to true.
- 'const_are_static': Boolean. Determines if constant variables are declared as 
                      static in the C code.
- 'precision': String, either 'double' or 'float'. Determines if real numbers are
               declared as type 'double' or type 'float'. Defaults to 'double'.
- 'time': Boolean. Determines if the solvers measure and return computation times.
          Execute `spcies('help', 'timing')` for more computation time information.

SPCIES receives these options by using a structure whose fields are named as the
above options list.
A structure with the default options can be obtained using:
`options = sp_utils.default_options();`

