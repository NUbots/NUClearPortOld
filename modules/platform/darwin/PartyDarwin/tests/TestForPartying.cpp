/*
 * This file is part of PartyDarwin.
 *
 * PartyDarwin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PartyDarwin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PartyDarwin.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <catch.hpp>

TEST_CASE("Test that the darwin will party", "[party][darwin]") {

    // Conditions
    bool darwinWillParty = true;
    bool darwinWillStopPartying = false;

    // Test case
    REQUIRE(darwinWillParty != darwinWillStopPartying);
}