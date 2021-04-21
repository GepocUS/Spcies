![GitHub release (latest by date including pre-releases)](https://img.shields.io/github/v/release/GepocUS/Spcies?include_prereleases&style=plastic)
![GitHub tag (latest SemVer pre-release)](https://img.shields.io/github/v/tag/GepocUS/Spcies?include_prereleases&style=plastic)
![GitHub](https://img.shields.io/github/license/GepocUS/Spcies?style=plastic)

# SPCIES

#### Suite of Predictive Controllers for Industrial Embedded Systems

This Matlab toolbox automatically generates solvers for _model predictive controllers_ (MPC) in different programming languages.

We support various MPC formulations, providing solvers based on _first order methods_. We focus of providing solvers that are suitable for their implementation in embedded systems, either because they are sparse, or because the MPC formulation is particularly suitable for this paradigm.

Currently, the toolbox generates plain C code or mex functions for Matlab.
For the compilation of the mex functions, a C compiler must be installed and interfaced with Matlab.
We assume the _gcc_ compiler is used in Linux for the compilation of the mex files.

Spcies is designed and developed by Pablo Krupa, Daniel Limon and Teodoro Alamo, from the University of Seville, Seville, Spain.

## Installation

Clone or download the repository into a directory $SPCIES$. Then, add the following folders to the Matlab path:

> * $SPCIES$
> * $SPCIES$/types
> * $SPCIES$/platforms
> * $SPCIES$/platforms/Matlab
> * $SPCIES$/generated_solvers

## Basic usage

A basic guide, examples, tutorials and in-depth documentation will be added during the initial development phase.
Users are encouraged to contact the authors by [email](mailto:pkrupa@us.es) or in the [discussion](https://github.com/GepocUS/Spcies/discussions) board if they have any questions or need any help with using the toolbox.

## Credits

If you use Spcies in your own research, please cite it using the following reference:

```
@misc{Spcies,
  author={Krupa, Pablo and Limon, Daniel and Alamo, Teodoro},
    title = {{Spcies: Suite of Predictive Controllers for Industrial Embedded Systems}},
    howpublished = {\url{https://github.com/GepocUS/Spcies}},
    month = {Dec},
    year = {2020}
}
```

## Contributing

All contributions are welcome. Please report any issues or bugs in the [Github issue tracker](https://github.com/GepocUS/Spcies/issues).

## Licence

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Notice

This repository is still in early stages of development. As such, it may be subject to significant changes in future releases.
