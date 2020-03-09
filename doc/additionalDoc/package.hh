/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-pattern-generator.
 * sot-pattern-generator is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-pattern-generator is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-pattern-generator.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
\mainpage

\section intro_sot-pattern-generator Introduction

This package wraps the walk pattern generator exposed by walkGenJrl (package
jrl-walkgen) for use in the stack of tasks. Hence, by using this package, you
can create several \ref dynamicgraph::Entity "entities" to control the walk
pattern of a humanoid robot.

\section req Requirements
This package has the following dependencies:
\li jrl-walkgen
\li jrl-dynamics
\li sot-core
\li hrp2Dynamics [optional]
\li hrp2-10-small [optional]

To download and install these packages, please visit
https://github.com/jrl-umi3218.

\section overview API overview
As most packages based on the dynamic-graph framework (see
https://github.com/jrl-umi3218/dynamic-graph), the functionality is exposed
through entities. Hence .so or .dll (dynamic-link) libraries are generated in
the dynamic-graph plugins directory.

The following entities are created by this package:\n
(all entites are placed in the namespace sot::)
\li sot::PatternGenerator
\li (to be continued)

See each entity's documentation page for more information (when available).

*/
