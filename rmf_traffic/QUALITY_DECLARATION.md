This document is a declaration of software quality for the `rmf_traffic` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_traffic` Quality Declaration

The package `rmf_traffic` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_traffic` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_traffic` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package.
Headers in any other folders are not installed and are considered private.

All launch files in the installed `launch` directory are considered part of the public API.

### API Stability Policy [1.iv]

`rmf_traffic` will not break public API within a major version number.

### ABI Stability Policy [1.v]

`rmf_traffic` will not break public ABI within a major version number.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`rmf_traffic` will not break public API or ABI within a released ROS distribution, i.e. no major releases into the same ROS distribution once that ROS distribution is released.

## Change Control Process [2]

`rmf_traffic` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_traffic` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_traffic` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.

The most recent CI results can be seen on [the workflow page](https://github.com/osrf/rmf_core/actions?query=workflow%3Abuild+branch%3Amaster).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_traffic` does not provide documentation.

### Public API Documentation [3.ii]

`rmf_traffic` documents its public API.
The documentation is not hosted.

### License [3.iii]

The license for `rmf_traffic` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmf_traffic`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

Each feature in `rmf_traffic` has corresponding tests which simulate typical usage.
They are located in the [`test`](https://github.com/osrf/rmf_core/tree/master/rmf_traffic/test) directory.
New features are required to have tests before being added.

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests are not run automatically.
They are located in the [`test`](https://github.com/osrf/rmf_core/tree/master/rmf_traffic/test) directory.

### Coverage [4.iii]

`rmf_traffic` does not track coverage statistics.

### Performance [4.iv]

`rmf_traffic` does not test performance.

### Linters and Static Analysis [4.v]

`rmf_traffic` does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

`rmf_traffic` uses a custom `uncrustify` configuration matching its coding style.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

Below are the required direct runtime ROS dependencies of `rmf_traffic` and their evaluations.

#### rmf_utils

`rmf_utils` is [**Quality Level 4**](https://github.com/osrf/rmf_core/blob/master/rmf_utils/QUALITY_DECLARATION.md).

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_traffic` has no optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

Below are the required direct runtime non-ROS dependencies of `rmf_traffic` and their evaluations.

#### eigen

`eigen` is taken to be **Quality Level 3** due to its wide-spread use, history, use of CI, and use of testing.

#### libccd-dev

`rmf_traffic` uses the [`libccd` library](https://github.com/danfis/libccd).
This is assumed to be **Quality Level 3** due to its provided documentation, use of testing, and version number above 1.0.0.

#### libfcl-dev

`rmf_traffic` uses [`fcl` (the Flexible Collision Library)](https://github.com/flexible-collision-library/fcl).
This is assumed to be **Quality Level 3** due to its provided feature documentation, use of testing, and version number above 1.0.0.

## Platform Support [6]

### Target platforms [6.i]

`rmf_traffic` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`rmf_traffic` supports ROS Eloquent.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
