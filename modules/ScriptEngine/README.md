Script Engine
=============

## Description

Loads script files containing robot movement instructions and allows them to
be executed.

## Usage

Scripts are JSON files containing a sequence of 'frames'. A frame lasts for a
set period of time and has instructions for the position and gain that servos
should be set to. Complex movements such as kicking or dancing are achieved
using many frames. Files are loaded using the configuration system, see the
Configuration section below for details of the format.

To execute a script, emit a `messages::ExecuteScriptByName` containing the
filename (without path) of the script to run and the time at which it should
start. Alternatively you can also emit a `messages::ExecuteScript` to run a
`messages::Script` directly.

## Consumes

* `messages::Configuration<Scripts>` containing the loaded scripts
* `messages::ExecuteScript` to run a script
* `messages::ExecuteScriptByName` to run a script from a file

## Emits

* `std::vector<messages::ServoWaypoint>` to set waypoints based on script

## Configuration

Script files are stored in a folder called `scripts` under the base config
directory and must have a .json extension. They are composed of an array of
'frames', each of which contains 'targets' for servos.

Frames are objects with the properties:

* "duration": the number of milliseconds that the frame will be active, how
  long until the next frame should begin
* "targets": an array of targets for servos to reach by the end of the frame

Target objects have:

* "id": the servo identifier as a string (see
  `messages::DarwinSensors::Servo::ID` for servo ID names)
* "position": the desired angle of the servo, in radians
* "gain": the desired motor gain

The basic structure of a script is thus:

    [
     {
     "duration": 150,
     "targets": [
                 { "id": "SERVO_ID", "position": 3.14159265, "gain": 100 },
                 { "id", "ANOTHER_SERVO", "position": -1.047197, "gain": 50 }
                ]
     }
    ]

## Dependencies

* The Config System module is required to read script files
* The Darwin Movement Manager is required to execute the waypoints defined in
  scripts

