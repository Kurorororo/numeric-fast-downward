# Numerical Fast Downward 

This is a work-in-progress repository for an extension of Numerical Fast Downward. 

Numerical Fast Downward (NFD), an extension of Fast Downward, has been originally developed by Johannes Aldinger and Bernhard Nebel. 

```
@inproceedings{aldinger2017interval,
  title={Interval based relaxation heuristics for numeric planning with action costs},
  author={Aldinger, Johannes and Nebel, Bernhard},
  booktitle={Joint German/Austrian Conference on Artificial Intelligence (K{\"u}nstliche Intelligenz)},
  pages={15--28},
  year={2017},
  organization={Springer}
}
```

Here is a list of the current additions:
  - numerical delete-relaxation IP/LP heuristics
  
    ```
    @inproceedings{piacentini2018linear,
      title={Linear and Integer Programming-Based Heuristics for Cost-Optimal Numeric Planning.},
      author={Piacentini, Chiara and Castro, Margarita P and Cir{\'e}, Andr{\'e} Augusto and Beck, J Christopher},
      year={2018}
    }
    ```

- compilation of planning problems into MIP models 

    ```
    @inproceedings{piacentini2018compiling,
      title={Compiling Optimal Numeric Planning to Mixed Integer Linear Programming.},
      author={Piacentini, Chiara and Castro, Margarita P and Cir{\'e}, Andr{\'e} Augusto and Beck, J Christopher},
      booktitle={ICAPS},
      pages={383--387},
      year={2018}
    }
    ```

- resource transformation of classical planning problems

    ```
    @inproceedings{wilhelm2018stubborn,
    title={On Stubborn Sets and Planning with Resources.},
    author={Wilhelm, Anna and Steinmetz, Marcel and Hoffmann, J{\"o}rg},
    booktitle={ICAPS},
    pages={288--297},
    year={2018}
    }
    ```

- subgoling-based hmax heuristic

    ```
    @article{scala2016heuristics,
    title={Heuristics for numeric planning via subgoaling},
    author={Scala, Enrico and Haslum, Patrik and Thi{\'e}baux, Sylvie and others},
    year={2016},
    publisher={AAAI Press}
    }
    ```

- landmark based heuristic

    ```
    @inproceedings{scala2017landmarks,
    title={Landmarks for Numeric Planning Problems.},
    author={Scala, Enrico and Haslum, Patrik and Magazzeni, Daniele and Thi{\'e}baux, Sylvie},
    booktitle={IJCAI},
    pages={4384--4390},
    year={2017}
    }
    ```

- numerical lm-cut heuristic
  
## Build 

### Install LPSolver

#### CPLEX
Install CPLEX.
IBM provides a free academic lincense: https://www.ibm.com/academic/home

Suppose that CPLEX is installed in `/opt/ibm/ILOG/CPLEX_Studio1210`.
Then, export the following environment variables.

```bash
export DOWNWARD_CPLEX_ROOT=/opt/ibm/ILOG/CPLEX_Studio1210/cplex
export DOWNWARD_CONCERT_ROOT=/opt/ibm/ILOG/CPLEX_Studio1210/concert
```

#### OSI
Instal open solver interface.

```bash
export DOWNWARD_COIN_ROOT=/path/to/osi
sudo apt install zlib1g-dev
wget http://www.coin-or.org/download/source/Osi/Osi-0.107.9.tgz
cd Osi-0.107.9
./configure CC="gcc"  CFLAGS="-pthread -Wno-long-long" \
            CXX="g++" CXXFLAGS="-pthread -Wno-long-long" \
            LDFLAGS="-L$DOWNWARD_CPLEX_ROOT/lib/x86-64_linux/static_pic" \
            --without-lapack --enable-static=no \
            --prefix="$DOWNWARD_COIN_ROOT" \
            --disable-bzlib \
            --with-cplex-incdir=$DOWNWARD_CPLEX_ROOT/include/ilcplex \
            --with-cplex-lib="-lcplex -lm -ldl"

make
sudo make install
cd ..
rm -rf Osi-0.107.9
rm Osi-0.107.9.tgz
```

#### build

```bash
cd numerical-fast-downward
./build.py release64
```

## Run

Follow the instructions of the original [Fast Downward](http://www.fast-downward.org/ObtainingAndRunningFastDownward) 

Quick summary:
```
fast-downward.py --build=release64 domain.pddl problem.pddl --search "astar(blind)
```

# ORIGINAL README (Fast Downward) 

Fast Downward is a domain-independent planning system.

The following directories are not part of Fast Downward as covered by this
license:
./src/search/ext

For the rest, the following license applies:

Copyright (C) 2003-2016 Malte Helmert
Copyright (C) 2008-2016 Gabriele Roeger
Copyright (C) 2012-2016 Florian Pommerening
Copyright (C) 2010-2015 Jendrik Seipp
Copyright (C) 2010, 2011, 2013-2015 Silvan Sievers
Copyright (C) 2013, 2015 Salome Simon
Copyright (C) 2014, 2015 Patrick von Reth
Copyright (C) 2015 Manuel Heusner, Thomas Keller
Copyright (C) 2009-2014 Erez Karpas
Copyright (C) 2014 Robert P. Goldman
Copyright (C) 2010-2012 Andrew Coles
Copyright (C) 2010, 2012 Patrik Haslum
Copyright (C) 2003-2011 Silvia Richter
Copyright (C) 2009-2011 Emil Keyder
Copyright (C) 2010, 2011 Moritz Gronbach, Manuela Ortlieb
Copyright (C) 2011 Vidal Alcázar Saiz, Michael Katz, Raz Nissim
Copyright (C) 2010 Moritz Goebelbecker
Copyright (C) 2007-2009 Matthias Westphal
Copyright (C) 2009 Christian Muise

Fast Downward is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Fast Downward is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.

For contact information see http://www.fast-downward.org/.
