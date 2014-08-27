# This file is part of NUbots Codebase.
#
# The NUbots Codebase is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# The NUbots Codebase is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
#
# Copyright 2013 NUBots <nubots@nubots.net>



cdef extern from "<nuclear>" namespace "NUClear::dsl":
    cdef cppclass Trigger[T]:
        pass
    cdef cppclass With[T]:
        pass


cdef extern from "<nuclear>" namespace "NUClear":
    cdef cppclass Reactor:
        pass

cdef class Cython:
    def __cinit__(self):
        print("An Amazing String!")

cdef api void hi():
    print("Please work!")