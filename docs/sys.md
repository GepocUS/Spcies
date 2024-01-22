# SPCIES -- Help page for topic 'sys'

The 'sys' structure is one the two basic structures used to 
define the MPC solver. It contains the information of the system.
The system is given by

> x(k+1) = A x(k) + B u(k),

where x(k) (of dimension n) is the state of the system and u(k) (of
dimension m) is the input of the system at sample time k.
The toolbox considers two types of constraints, either box constraint

> LBx <= x(k) <= UBx,
> LBu <= u(k) <= UBu,

or coupled input-state constraints

> LBy <= E x(k) + F u(k) <= UBy,

where the dimension of LBy and UBy is p.

The toolbox may also consider an operating point (x0, u0) and scaling
vectors Nx and Nu when working in "engineering" mode.
See `spcies('help', 'in_engineering')` or the `tutorial t03_real_systems.m`

Possible fields of 'sys':

- 'A', 'B': Matrices of the state-space model. Dimensions nxn and nxm.
- 'LBx', 'UBx': Vectors of dimension n. Box constraints for the state.
                Assumed to satisfy LBx < UBx.
- 'LBu', 'UBu': Vectors of dimension m. Box constraints for the input.
                Assumed to satisfy LBu < UBu.
- 'E', 'F': Matrices of the coupled constraints. Dimensions pxn and pxm.
- 'LBy', 'UBy': Vectors of dimension p. Box constraints for the coupled
                constraints. Assumed to satisfy LBy < UBy.

Instead of a structure, 'sys' can also be an instance of the class `ssModel`
from the GepocToolbox, available at: https://github.com/GepocUS/GepocToolbox
In that case, the above fields correspond to the following properties of ssModel:

A->A, B->Bu, LBx->LBx, UBx->UBx, LBu->LBu, UBu->UBu, LBy->LBy, UBy->UBy, E->Cc, F->DCu
