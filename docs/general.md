# SPCIES -- Suite of Predictive Controllers for Industrial Embedded Systems

SPCIES is a Matlab toolbox for the generation of sparse solvers for various
MPC controllers. It focuses on the generation of solvers in the C programming
language, but can also compile the solver using Matlab's MEX compiler for its
use inside Matlab.

For basic usage of the toolbox, we recommend following and executing the tutorials
provided in the directory `examples/` of the toolbox (execute the command
`spcies_get_root_directory` if you are unsure about the location of the toolbox).
Tutorials are named as `tXX_name.m`. We recommend starting with `t00_basic_tutorial.m`
and then continue in ascending order.

For specific documentation on a particular topic, execute: `spcies('help', 'topic_name')`.
For the list of main topics names please execute: `spcies('help', 'topics')`.

For any further information or help please contact the developers at:
https://github.com/GepocUS/Spcies

We also encourage the user to provide their feedback or report any bugs as an Issues
in the GitHub repository.
