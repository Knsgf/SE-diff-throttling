# Description
This highly experimental (read: sloppily written) plugin adds engine torque, engine rotational controls 
and automatic differential throttling to Space Engineers.

## Notes:
* This plugin is **single-player only**,
* On landed and docked ships, all engines and gyroscopes should be switched off. Failure to do so may result in sudden jolt upon lift-off or undocking,
* Because this plugin uses thrust override, it is not recommended to mix throttled and unthrotttled engines on the same side of a ship,
* **BEWARE OF LANDING GEAR LOCK!** Thrust override on a carrier tends to bug out when smaller ship locks itself to it. To avoid this,
always use landing gear *on the carrier* to secure smaller craft, not the other way around.
  
## Installation
1. Download and install Visual Studio 2015,
2. Download and unpack the repository,
3. Change ReferencePath in ttdt\ttdt.csproj.user to point to your SpaceEngineers\bin64 folder,
4. Compile the solution in Visual Studio,
5. Copy ttdt.dll into SpaceEngineers\bin64 directory,
6. In Steam, right-click on Space Engineers in your library and choose "Properties",
7. Click "SET LAUNCH OPTIONS...", then type the following text without quote marks:
   "-plugin ttdt.dll",
8. Click "OK" and close "Properties" window. The plugin will be automatically loaded whenever you start SE.

## Deinstallation
1. In Steam, right-click on Space Engineers in your library and choose "Properties",
2. Click "SET LAUNCH OPTIONS..." and remove the following text:
   "-plugin ttdt.dll",
3. Click "OK" and close "Properties" window,
4. Delete ttdt.dll in your SpaceEngineers\bin64 folder.

## Usage
By default, all thrusters exert torque on a ship, but do not participate in steering or stabilisation of the vessel.
To assign a thruster to engine control, open Control Panel and add [RCS] text anywhere in thuster's name. 
The plugin is case-insensitive, so [RCS] and [rcs] will both work.
