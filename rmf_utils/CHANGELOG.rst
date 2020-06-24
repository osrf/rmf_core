^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-23)
------------------
* Basic utilities for use in the `rmf_core` packages
  * `impl_ptr` - A smart pointer that helps implement a PIMPL-pattern
  * `clone_ptr` - A smart pointer with cloning semantics (copying the pointer instance will clone the underlying object)
  * `optional` - A stopgap measure to get the features of `std::optional` before C++17 is available
  * `Modular` - A class for comparing integral version numbers that may wrap around due to integer overflow
* Contributors: Grey, Luca Della Vedova, Marco A. Gutiérrez, Michael X. Grey, Morgan Quigley, Yadu, Yadunund
