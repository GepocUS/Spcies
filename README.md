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

Spcies is developed and maintained by the [Gepoc](https://grupo.us.es/gepoc/) research group from the Universidad de Sevilla, Seville, Spain.

## Installation

Download or clone the desired version of the toolbox into a directory 'spcies_root/'.
For normal use, we recommend downloading the latest [release](https://github.com/GepocUS/Spcies/releases) of the toolbox.
Then move Matlab's current folder to 'spcies_root/' and execute `spcies('install')` in the command window.

Alternatively, you can manually add the following folders to Malab's path.

> * spcies_root/
> * spcies_root/formulations/
> * spcies_root/classes/
> * spcies_root/platforms/
> * spcies_root/platforms/Matlab/
> * spcies_root/platforms/Matlab/personal/
> * spcies_root/generated_solvers/
> * spcies_root/tests/

To uninstall simply execute `spcies('uninstall')` in the command window. You can also manually remove the above folders from Matlab's path.

To update the toolbox we recommend uninstalling the current install and then downloading and reinstalling the new version.
We remark that this toolbox is still in early stages of development. As such, it may be subject to significant changes in future releases.
Thus, backward compatibility may be lost when updating the toolbox to newer releases or versions.

## Usage and documentation

We recommend reading through and executing the [basic tutorial](examples/t00_basic_tutorial.m) script located in [examples](examples). It explains the basic usage of the toolbox, including how to generate and use the solvers. We also recommend reading though and executing the other tutorials in ascending numerical order.

Documentation of the toolbox is provided by `spcies('help')`, where documentation for specific topics can be obtained by using `spcies('help', 'topic')`. For the list of topics with available documentation execute `spcies('help', 'topics')`.

The user is also encouraged to execute `help spcies` for documentation on the commands available in `spcies()`.

Users are encouraged to contact the authors by [email](mailto:pkrupa@us.es) or to participate in the [discussion](https://github.com/GepocUS/Spcies/discussions) board if they have any questions or need any help with using the toolbox.

## Credits

If you use Spcies in your own research, please cite it using the following reference:

```
@misc{Spcies,
  author={Krupa, Pablo and Gracia, Victor and Limon, Daniel and Alamo, Teodoro},
    title = {{Spcies: Suite of Predictive Controllers for Industrial Embedded Systems}},
    howpublished = {\url{https://github.com/GepocUS/Spcies}},
    month = {Dec},
    year = {2020}
}
```

## Contributing

All contributions are welcome. Please report any issues or bugs in the [Github issue tracker](https://github.com/GepocUS/Spcies/issues).

Contributions are also welcome as [Pull Requests](https://github.com/GepocUS/Spcies/pulls). Developers wishing to contribute new code or modification to existing code are encouraged to clone the repository and work on the 'develop' branch, which contains the latest (possibly non-stable) version of the toolbox. Merges to the 'master' branch are reserved for the latest stable version of the toolbox, typically associated with a new version number and release.
Documentation for developers can be obtained by executing `spcies('help', 'develop')`.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
