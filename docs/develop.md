# SPCIES -- Help page for topic 'develop'

This help page contains basic help and guidelines for developers.
Contributions to the toolbox are welcome as pull requests in:
https://github.com/GepocUS/Spcies
Thus, basic knowledge of git and GitHub are required. See "Using git" below.

## Structure of the toolbox

```
    spcies_root/
    ├── +sp_utils/
    │   └── util_function.m
    ├── classes/
    │   ├── Spcies_constructor.m
    │   └── Spcies_problem.m
    ├── docs/
    │   └── topic.txt
    ├── examples/
    │   └── tXX_tutorial_name.m
    ├── generated_solvers/
    ├── platforms/
    │   ├── +C_code/
    │   └── Matlab/
    │       ├── personal/
    │       └── solver_formulation_method.m
    ├── tests/
    │   ├── spcies_tester.m
    │   └── test_xx.m
    ├── formulations/
    │   └── +formulation/
    │       ├── code_formulation_method_submethod_C.c
    │       ├── compute_formulation_method_submethod_ingredients.m
    │       ├── cons_formulation_method_submethod_C.m
    │       ├── cons_formulation_method_submethod_Matlab.m
    │       ├── def_options_formulation_method_submethod.m
    │       ├── header_formulation_method_submethod_C.h
    │       └── struct_formulation_method_submethod_C_Matlab.cLICENSE
    ├── snippets/
    ├── LICENSE
    ├── README.md
    ├── spcies.m
    ├── spcies_gen_controller.m
    └── spcies_public_functions.m
```

### Explanation of each directory/file:

- spcies.m -> One of the main entry points of the toolbox. Used for displaying
  information to the user and for executing some useful commands.
- spcies_gen_controller.m -> The entry point for solver generation. The user calls
  this function for generating MPC solvers.
  This function can also be called as `spcies('gen', args)`.
- spcies_public_functions.m -> Any other function that can be called by the user.
  By convention they are named as `spcies_function_name.m`.
  Functions that are not indented for public use should be put into `+sp_utils/`.
  In general, all of these public functions can be called using `spcies('name', [args])`.
- classes/ -> Folder that contains class definitions used by the toolbox. They are not
  intended for public use, but instead used by the toolbox itself to define and generate
  files. The two main classes are:
    - `Spcies_constructor.m` -> Class that is used for the generation of the MPC solvers.
      An instance is created in function `spcies_gen_controller()`.
    - `Spcies_problem.m` -> Class that is used to define the code-generation procedure.
      Used in the class Spcies_constructor to define the files that need to be generated.
- `README.md` -> General README file for the GitHub repository.
- `LICENSE` -> LICENSE of the toolbox.
- `+sp_utils/` -> Directory containing any internal toolbox function, i.e., private functions
  that are not intended for the user.
  These functions don't have any particular naming convention.
- `docs/` -> Folder containing the help pages for each each topic. These help pages can
  be displayed using `spcies('help', 'topic')`. The files in this directory are named as
  'topic.txt', where 'topic' is what the user should write in `spcies('help', 'topic')`.
- `examples/` -> Folder containing examples and tutorial. The tutorials are .m scripts
  that follow the naming convention 'tXX_name.m', where 'XX' are numbers in ascending
  order, starting from 01 (02, 03, etc.). The number is meant as an indication of the
  recommended order in which to follow the tutorials.
- `generated_solvers/` -> Default directory where SPCIES saves the generated files and 
  where compiled MEX functions are stored. This directory is added to the Matlab path
  in the toolbox installation.
- `platforms/` -> Directory containing functions used for code-generation in different
  platforms. Currently only C programming language is supported.
    - `+C_code/` -> Functions used for code generation in the C programming language.
    - `Matlab/` -> Versions of the solvers available in the toolbox programmed in Matlab
      programming language. They are useful for development and debugging of the C solvers.
- `tests/` -> Suite of tests of the toolbox. Mainly used as "unit tests" for development.
    - `spcies_tester.m` -> Entry point to running the tests. Can be called using spcies('test')
    - `test_xx.m` -> All the tests that can be run using spcies_tester().
- `formulations/` -> Directory that contains a folder for each of the MPC formulations available in
  the toolbox. Each formulation is given a name. The folder for each formulation follows the
  naming convention `+formulation/`, where 'formulation' is the name of the formulation. Variations of
  a same MPC formulation can share the same `+formulation/` folder, e.g., formulations 'HMPC' and
  'ellipHMPC' share the '+HMPC' folder.
  Each `+formulation/` folder must contain a series of functions and files that form the basis for 
  the generation of the solvers. These functions follow a strict naming convention (otherwise
  they will not be found by `Spcies_constructor`) where 'formulation' and 'method' and 'submethod' should
  be substituted by their corresponding names as given to spcies_gen_controller().
    - `code_formulation_method_submethod_C.c` -> Template of C code of the solver. See "Template C" below.
    - `compute_formulation_method_submethod_ingredients.m` -> Computes the offline ingredients of the solver.
    - `cons_formulation_method_submethod_C.m` -> Performs the variable declarations of the solver for C code.
    - `cons_formulation_method_submethod_Matlab.m` -> Same as before but when generating for Matlab 'platform'.
    - `def_options_formulation_method_submethod.m` -> Returns the default solver options.
    - `header_formulation_method_submethod_C.h` -> Template of the C header. See "Template C" below.
    - `struct_formulation_method_submethod_C_Matlab.c` -> Template of the C mexFunction file.
      See "Template MEX functions" below.
  If a formulation/method does not have a submethod or has a submethod without a name (default method), then
  the naming convention is to eliminate the '_submethod'. For example, 'laxMPC' with 'ADMM' and now
  submethod (i.e., default ADMM algorithm) would be named as `code_laxMPC_ADMM_C.c`.
- `snippets/` -> This directory contains template snippets that can be inserted into files in
  the code generation process. See "Automatic snippet insertion in code" below.

## Template C
    
SPCIES generates solvers by constructing offline all the ingredients required online and inserting
them into a .c file that contains the code of the solver. That is, each formulation-method-submethod
trio has a .c file named code_formulation_method_submethod_C.c (located in the corresponding
`formulations/+formulation/` folder) that contains the code of the solver and is missing the variables
(defines, constants and variables).
Instead, the template contains tags that will be substituted by the corresponding text in the file
generation process. Tags follow the naming convention `$TAG_NAME$`.
Default tags are:

- `$INSERT_CONSTANTS$` -> Constant variables will be inserted here.
- `$INSERT_VARIABLES$` -> Non constant variables will be inserted here.
- `$INSERT_DEFINES$` -> ``#define`` C variables will be inserted here.
- `$INSERT_NAME$` -> The name of the generated files (determined by `options.save_name`)
- `$INSERT_PATH$` -> The path of the file will be inserted here

Other tags can be defined in `cons_formulation_method_submethod_C.m` or
`cons_formulation_method_submethod_Matlab.m`.

A header file for the code_formulation_method_submethod_C.c code is also required. The naming convention is
`header_formulation_method_submethod_C.h`.
As in the .c file, tags are substituted by the text defined in the 'cons_' functions or `Spcies_constructor`.

## Template MEX functions

For the generation and compilation of Matlab MEX functions, another file is required. The MEX functions
use the same .c solver, but require a .c file containing a `mexFunction()` as Matlab's entry point.
The naming convention for this file is `struct_formulation_method_submethod_C_Matlab.c`.
Tags may also be used in this file.
        
## Generating a new solver for a new or existing MPC formulation
    
For each trio formulation-method-submethod, the developer needs to create a new copy of the functions
described in above section "Structure of the toolbox" within the `+formulations/` folders, i.e., to create:

- `code_formulation_method_submethod_C.c`
- `compute_formulation_method_submethod_ingredients.m`
- `cons_formulation_method_submethod_C.m`
- `cons_formulation_method_submethod_Matlab.m`
- `def_options_formulation_method_submethod.m`
- `header_formulation_method_submethod_C.h`
- `struct_formulation_method_submethod_C_Matlab.c`

The easiest way to go about this is to use an existing set of files as a guideline and make the
necessary changes for the new formulation-method-submethod.

The developer may also need to update `sp_utils.default_options.m` to indicate the default method and
submethod of a new formulation.

Finally, the developer should add or modify the docs of the toolbox, i.e., the files in `docs/` to
add any new documentation or update any documentation that is now incorrect or incomplete.
Note that the doc files are written in markdown to facilitate readability outside of Matlab.
Thus, markdown system should be followed.

## Automatic snippet insertion in code

Similar to the tags (e.g., `$INSERT_NAME$`), SPCIES offers an automatic way of inserting snippets
or blocks of code into a .c or .h template (See "Template C").

In a .c or .h, a line such as:
`spcies_snippet_name()`;
will be substituted by the text in file `spcies_root/snippets/name.extension`, where '.extension'
will be the extension of the file being generated (.c or .h).

This is useful for sharing blocks of code that are used in more than one solver. For example, for
the functions used to measure computation times.

## Personal folders
    
The folders `spcies_root/platforms/Matlab/personal/` and `spcies_root/formulations/+personal/` are ignored
by `.gitignore` file. They can be used to store functions and templates that are still in early development
before including them in the `spcies_root/platforms/Matlab` or `spcies_root/formulations/+formulation/`
folder for committing it in git.

## Using git

We use a 'master' branch for the current production version of the toolbox.
Each merge to the 'master' branch is typically associated with a new version
number and release in the GitHub platform.

We use a 'develop' branch for changes being made towards the next release.
Pull requests should be either made to the 'develop' branch or to some 'feature'
subbranch of 'develop'.

Git commit messages should use start with one of the following tags:

- 'feat:' when adding or working on a new feature.
- 'fix:' or 'bug:' when fixing some issue or bug.
- 'update:' when updating some formulatio/method to the latest changes or updates.
- 'refactor:' when refactoring code that neither fixed bug nor adds a feature.
- 'docs:' when updating documentation or README files.
- 'style:' when making changes in style/formatting.
- 'test:' when working on tests files or adding new tests.
- 'perf:' when working on performance improvements.
- 'revert:' when reverting a previous commit.

'feat:' is also used when adding a new solver, e.g., "feat: added laxMPC ADMM-based solver"

Changes made on a particular solver should be indicated using parentheses when applicable.
For example: "perf (laxMPC-ADMM): Improved computation of dual variables"
