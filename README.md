sot-pattern-generator
=====================

[![Building Status](https://travis-ci.org/stack-of-tasks/sot-pattern-generator.svg?branch=master)](https://travis-ci.org/stack-of-tasks/sot-pattern-generator)
[![Pipeline status](https://gitlab.laas.fr/stack-of-tasks/sot-pattern-generator/badges/master/pipeline.svg)](https://gepgitlab.laas.fr/stack-of-tasks/sot-pattern-generator/commits/master)
[![Coverage report](https://gitlab.laas.fr/stack-of-tasks/sot-pattern-generator/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/stack-of-tasks/sot-pattern-generator/master/coverage/)

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
