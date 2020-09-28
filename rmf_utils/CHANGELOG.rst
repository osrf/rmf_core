^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-09-24)
------------------
* Replace rmf_utils::optional with std::optional (`#177 <https://github.com/osrf/rmf_core/issues/177>`_)
* Foxy Support (`#133 <https://github.com/osrf/rmf_core/issues/133>`_)
* Remove ros2 dependency (`#142 <https://github.com/osrf/rmf_core/issues/142>`_)
* Contributors: Aaron Chong, Grey, Yadu

1.0.0 (2020-06-23)
------------------
* Basic utilities for use in the `rmf_core` packages
    * `impl_ptr` - A smart pointer that helps implement a PIMPL-pattern
    * `clone_ptr` - A smart pointer with cloning semantics (copying the pointer instance will clone the underlying object)
    * `optional` - A stopgap measure to get the features of `std::optional` before C++17 is available
    * `Modular` - A class for comparing integral version numbers that may wrap around due to integer overflow
* Contributors: Grey, Luca Della Vedova, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley, Yadu, Yadunund
