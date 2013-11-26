/*
 * This file is part of ConsoleLogHandler.
 *
 * ConsoleLogHandler is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ConsoleLogHandler is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ConsoleLogHandler.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_CONSOLELOGHANDLER_H
#define MODULES_CONSOLELOGHANDLER_H

#include <NUClear.h>

namespace modules {

    /**
     * TODO document
     *
     * @author Jake Woods
     */
    class ConsoleLogHandler : public NUClear::Reactor {
    public:
        explicit ConsoleLogHandler(std::unique_ptr<NUClear::Environment> environment);
    };
}
#endif

