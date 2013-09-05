sot-pattern-generator
=====================

[![Build Status](https://travis-ci.org/stack-of-tasks/sot-pattern-generator.png?branch=master)](https://travis-ci.org/stack-of-tasks/sot-pattern-generator)
[![Coverage Status](https://coveralls.io/repos/stack-of-tasks/sot-pattern-generator/badge.png)](https://coveralls.io/r/stack-of-tasks/sot-pattern-generator)


This software provides jrl-walkgen bindings for the dynamic-graph
package. It allows the computation of whole-body biped walk
trajectories.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.
