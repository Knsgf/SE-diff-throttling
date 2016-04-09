**Notice:** this release has been superseded by workshop mod available here:
http://steamcommunity.com/sharedfiles/filedetails/?id=660333966




# Original description
This proof-of-concept (read: sloppily written) plugin adds thruster torque, engine steering 
and automatic differential throttling to Space Engineers.

## Notes
* This plugin is **single-player only**,
* On landed and docked ships, either all engines and gyroscopes should be switched off or **main cockpit** should be unchecked. Failure to do so may result in sudden jolt upon lift-off or undocking,
* Because this plugin uses thrust override, it is not recommended to mix throttled and unthrotttled engines on the same side of a ship,
* **BEWARE OF LANDING GEAR AND ROTOR/PISTON SAFETY LOCKS!** Planet gravity affecting a carrier tends to bug out when smaller ship locks itself to it; 
  the same thing also happens when rotors and pistons are locked.
  
## Installation
1. Download the following file:
   https://github.com/Knsgf/SE-diff-throttling/blob/master/ttdt.dll 
   and save it into **Steam\steamapps\common\SpaceEngineers\Bin64\** directory 
   (you may need administrator rights to do so),
2. In Steam, right-click on **Space Engineers** in your library and choose **Properties**,
3. Click **SET LAUNCH OPTIONS...**, then type the following text:
   **-plugin ttdt.dll**,
4. Click **OK** and close **Properties** window. The plugin will be automatically loaded whenever you start SE.

## Deinstallation
1. In Steam, right-click on Space Engineers in your library and choose **Properties**,
2. Click **SET LAUNCH OPTIONS...** and remove the following text:
   **-plugin ttdt.dll**,
3. Click **OK** and close **Properties** window,
4. Delete **ttdt.dll** in your **Steam\steamapps\common\SpaceEngineers\Bin64\** folder.

## Usage
By default, all thrusters exert torque on a ship, but do not participate in steering or stabilisation of the vessel.
To assign a thruster to engine control, open Control Panel and add **[RCS]** text anywhere in thuster's name. 
The plugin is case-insensitive, so **[RCS]** and **[rcs]** will both work.

#### Main cockpit
The system behaves differently depending on whether main cockpit is set:
* If main cockpit is checked, then the system will actively try to hold established attitude. While in gravity, the ship will hover in place,
* If main cockpit is not checked, then the system will only reduce spin instead of nullifying it; this is useful e.g. for wheeled vehicles.
   While in gravity, the ship will slowly descend.

#### Remotely controlled drones without cockpit
On RC drone the stabilisation behaves as follows:
* When a drone is under player's control and **Show horizon and altitude** box in RC block is checked, then the system will actively try to hold established attitude. 
  While in gravity, the drone will hover in place,
* When a drone is not under player's control or when **Show horizon and altitude** is not checked, then the system will only reduce spin instead of nullifying it.
  While in gravity, the drone will slowly descend.
   
#### Thrust calibration
In addition to usual dynamic stabiliser, which adjusts thrusters in real time in response to angular velocity, it is now possible to 
pre-calibrate thrust levels by adding **[STAT]** tag to thrusters' names. The primary advantage of static calibration is less sway during flight 
as well as better feedback on thruster balance; however ships with calibrated engines tend to accelerate and slow down worse than same 
vessels relying solely on dynamic control. It is also possible for calibration to fail (e.g. when all engines are only on one side of CoM);
the system will revert to dynamic solution in the latter case.

## Compiling the source
1. Download and install Visual Studio 2015,
2. Download and unpack the repository,
3. Change **ReferencePath** in **ttdt\ttdt.csproj.user** to point to your **Steam\steamapps\common\SpaceEngineers\Bin64\** folder,
4. Open the solution in Visual Studio, set build to **Release** and compile it,
5. Copy **ttdt.dll** into **Steam\steamapps\common\SpaceEngineers\Bin64\** directory.

# Acknowledgements
Special thanks to:
   Agnar, Berun, Blitz4532, bos187, bruin_o9er, busboy999, convict1103, Creeping Wolf, DaMasta, Dan2D3D, deadok, DreamAssembly!, Dwarf-Lord Pangolin, 
   Dynamite Knight, EvanBaxter, grimsage, Gusset_Tugger, Idartacus [Farstar Gamer], IIN8II, izzik, Lord Commissar, M3rchantOfD3ath,
   morty, [MPW] ColTeH, Rayne, Shirako1988, Space_Engineers7, Sparty, [ST-R] Solutech [ST], trents2cool, Tristavius, TronicE, TumbleTV