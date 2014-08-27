/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

// I need this because CYTHON ARE MORONS
#ifndef DL_IMPORT
  #define DL_IMPORT(t) t
#endif

#include "CythonTest.h"
#include <Python.h>
#include "Interface.h"

namespace modules {
namespace debug {

    std::shared_ptr<PyObject> initModule(NUClear::Reactor* reactor) {
        // Initialise Python
        if(!Py_IsInitialized()) {
            Py_InitializeEx(0);
        }

        // Initialise this modules things
        PyInit_Interface();

        // Build a python reactor interface
        return std::shared_ptr<PyObject>(buildCythonTest(reactor));
    }

    CythonTest::CythonTest(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    , interface(initModule(this)) {}

}
}
