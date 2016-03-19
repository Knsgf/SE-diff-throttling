# Description
This highly experimental (read: sloppily written) plugin adds engine torque, engine steering 
and automatic differential throttling to Space Engineers.

## Notes
* This plugin is **single-player only**,
* On landed and docked ships, either all engines should be switched off or "Main Cockpit" should be unchecked. Failure to do so may result in sudden jolt upon lift-off or undocking,
* Because this plugin uses thrust override, it is not recommended to mix throttled and unthrotttled engines on the same side of a ship,
* **BEWARE OF LANDING GEAR AND ROTOR/PISTON SAFETY LOCKS!** Planet gravity affecting a carrier tends to bug out when smaller ship locks itself to it; 
  the same thing also happens when rotors and pistons are locked.
  
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

### Main cockpit
The system behaves differently depending on whether main cockpit is set:
* If main cockpit is checked, then the system will actively try to hold established attitude. While in gravity, the ship will hover in place,
* If main cockpit is not checked, then the system will only reduce spin instead of nullifying it; this is useful e.g. for wheeled vehicles.
   While in gravity, the ship will slowly descend.
   
# Acknowledgements
Special thanks to:
   Agnar, Blitz4532, bos187, bruin_o9er, busboy999, Creeping Wolf, DaMasta, deadok, DreamAssembly!, Dwarf-Lord Pangolin, Dynamite Knight, Gusset_Tugger, 
   Idartacus [Farstar Gamer], izzik, Space_Engineers7, trents2cool, Tristavius, TumbleTV