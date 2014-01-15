/*
 * This file is part of SignalCatcher.
 *
 * SignalCatcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SignalCatcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SignalCatcher.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_SUPPORT_SEGMENTATIONFAULT_H
#define MESSAGES_SUPPORT_SEGMENTATIONFAULT_H

#include <exception>

namespace messages {
    namespace support {

        /**
         * Is thrown when a segmentation fault happens and SignalCatcher is installed
         *
         * @author Trent Houliston
         */
        class SegmentationFault : public std::exception {};
        
    }  // support
}  // messages

#endif  // MESSAGES_SUPPORT_SEGMENTATIONFAULT_H
