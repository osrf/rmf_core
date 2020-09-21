This document is a declaration of software quality for the `rmf_door_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_door_msgs` Quality Declaration

The package `rmf_door_msgs` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_door_msgs` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_door_msgs` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message definition files located the `msg` directory are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`rmf_door_msgs` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`rmf_door_msgs` does not contain any C or C++ code and therefore will not affect ABI stability.

## Change Control Process [2]

`rmf_door_msgs` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_door_msgs` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_door_msgs` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_door_msgs` has basic comments in the message definition files, but no list of messages or usage guide is provided.
New messages require their own documentation in order to be added.

### Public API Documentation [3.ii]

`rmf_door_msgs` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `rmf_door_msgs` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

There are no source files that are currently copyrighted in this package so files are not checked for abbreviated license statements.

### Copyright Statement [3.iv]

There are no copyrighted source files in this package.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 3 lists.

## Testing [4]

### Feature Testing [4.i]

`rmf_door_msgs` is a package providing strictly message definitions and therefore does not require associated tests.

### Public API Testing [4.ii]

`rmf_door_msgs` is a package providing strictly message definitions and therefore does not require associated tests.

### Coverage [4.iii]

`rmf_door_msgs` is a package providing strictly message definitions and therefore has no coverage requirements.

### Performance [4.iv]

`rmf_door_msgs` is a package providing strictly message definitions and therefore has no performance requirements.

### Linters and Static Analysis [4.v]

`rmf_door_msgs` does not use the standard linters and static analysis tools for its generated C++ and Python code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`rmf_door_msgs` has the following runtime ROS dependencies, which are at or above Quality Level 3:
* `builtin_interfaces`: [QL 3](https://github.com/ros2/rcl_interfaces/tree/master/builtin_interfaces/QUALITY_DECLARATION.md)
* `rosidl_default_runtime` [QL 3](https://github.com/ros2/rosidl_defaults/tree/master/rosidl_default_runtime/QUALITY_DECLARATION.md)

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_door_msgs` does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`rmf_door_msgs` does not have any runtime non-ROS dependencies.

## Platform Support [6]

As a pure message definitions package, `rmf_door_msgs` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), but does not currently test each change against all of them.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
