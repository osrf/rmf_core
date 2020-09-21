This document is a declaration of software quality for the `rmf_cmake_uncrustify` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_cmake_uncrustify` Quality Declaration

The package `rmf_cmake_uncrustify` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_cmake_uncrustify` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_cmake_uncrustify` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

The CMake macros provided in the `cmake` directory are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`rmf_cmake_uncrustify` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`rmf_cmake_uncrustify` does not contain any C or C++ code and therefore will not affect ABI stability.

## Change Control Process [2]

`rmf_cmake_uncrustify` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_cmake_uncrustify` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_cmake_uncrustify` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_cmake_uncrustify` has documentation for each CMake macro in the macro definition file.
A usage guide is provided in the `doc` directory.

### Public API Documentation [3.ii]

`rmf_cmake_uncrustify` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `rmf_cmake_uncrustify` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

The CMake files contain copyright headers, although this is not automatically verified using linters.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmf_cmake_uncrustify`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 3 lists.

## Testing [4]

### Feature Testing [4.i]

`rmf_cmake_uncrustify` is a package providing strictly CMake macros and therefore does not require associated tests.

### Public API Testing [4.ii]

`rmf_cmake_uncrustify` is a package providing strictly CMake macros and therefore does not require associated tests.

### Coverage [4.iii]

`rmf_cmake_uncrustify` is a package providing strictly CMake macros and therefore has no coverage requirements.

### Performance [4.iv]

`rmf_cmake_uncrustify` is a package providing strictly CMake macros and therefore has no performance requirements.

### Linters and Static Analysis [4.v]

`rmf_cmake_uncrustify` uses the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`rmf_cmake_uncrustify` has no required runtime dependencies.

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_cmake_uncrustify` has no optional runtime dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`rmf_cmake_uncrustify` does not have any runtime non-ROS dependencies.

## Platform Support [6]

As a pure CMake package, `rmf_cmake_uncrustify` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), but does not currently test each change against all of them.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
