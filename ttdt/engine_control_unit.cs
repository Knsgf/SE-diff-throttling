using System;
using System.Collections.Generic;
using System.Diagnostics;
//using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Cube;
using Sandbox.Game.GameSystems;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Utils;
using VRageMath;

namespace thruster_torque_and_differential_throttling
{
    class engine_control_unit: IDisposable
    {
        enum thrust_dir { fore = 0, aft = 1, starboard = 2, port = 3, dorsal = 4, ventral = 5 };
        struct thruster_info
        {
            public MyThrust thruster_ref;
            public Vector3 max_force;
            public Vector3 max_torque;
            public Vector3 grid_centre_pos;
            public Vector3 static_moment;
            public Vector3 CoM_offset;
        };

        private MyCubeGrid              grid;
        private MyGridGyroSystem        gyro_control;
        private MyEntityThrustComponent thrust_control;
        private IMyGridTerminalSystem   grid_control;
        private Dictionary<MyThrust, thruster_info>[] controlled_thrusters =
        {
            new Dictionary<MyThrust, thruster_info>(),   // fore
            new Dictionary<MyThrust, thruster_info>(),   // aft
            new Dictionary<MyThrust, thruster_info>(),   // starboard
            new Dictionary<MyThrust, thruster_info>(),   // port
            new Dictionary<MyThrust, thruster_info>(),   // dorsal
            new Dictionary<MyThrust, thruster_info>()    // ventral
        };
        private Dictionary<MyThrust, thruster_info> uncontrolled_thrusters = new Dictionary<MyThrust, thruster_info>();

        private bool disposed = false;
        private Vector3D grid_CoM_location;
        private MatrixD inverse_world_transform;

        [Conditional("DEBUG")]
        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT \t\tengine_control_unit<{0} [{1}]>.{2}(): {3}", grid.DisplayName, grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (var cur_direction in controlled_thrusters)
                num_controlled_thrusters += cur_direction.Count;
            MyLog.Default.WriteLine(string.Format("TT&DT \t\ttotal thrusters: {0} ({1}/{2}/{3}/{4}/{5}/{6} controlled, {7} uncontrolled)", 
                uncontrolled_thrusters.Count + num_controlled_thrusters,
                controlled_thrusters[(int)thrust_dir.fore     ].Count,
                controlled_thrusters[(int)thrust_dir.aft      ].Count,
                controlled_thrusters[(int)thrust_dir.starboard].Count,
                controlled_thrusters[(int)thrust_dir.port     ].Count,
                controlled_thrusters[(int)thrust_dir.dorsal   ].Count,
                controlled_thrusters[(int)thrust_dir.ventral  ].Count,
                uncontrolled_thrusters.Count));
        }

        private thrust_dir get_nozzle_orientation(MyThrust thruster)
        {
            Vector3I dir_vector = thruster.ThrustForwardVector;
            if (dir_vector == Vector3I.Forward)
                return thrust_dir.fore;
            if (dir_vector == Vector3I.Backward)
                return thrust_dir.aft;
            if (dir_vector == Vector3I.Left)
                return thrust_dir.port;
            if (dir_vector == Vector3I.Right)
                return thrust_dir.starboard;
            if (dir_vector == Vector3I.Up)
                return thrust_dir.dorsal;
            if (dir_vector == Vector3I.Down)
                return thrust_dir.ventral;
            throw new ArgumentException("Thruster " + thruster.CustomName  + " is not grid-aligned");
        }

        private void assign_thruster(MyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.thruster_ref    = thruster;
            new_thruster.grid_centre_pos = (thruster.Min + thruster.Max) * (grid.GridSize / 2.0f);
            new_thruster.max_force       = thruster.ThrustForce;
            new_thruster.static_moment   = new_thruster.grid_centre_pos * new_thruster.max_force.Length();
            new_thruster.CoM_offset      = new_thruster.grid_centre_pos - grid_CoM_location;
            new_thruster.max_torque      = Vector3.Cross(new_thruster.CoM_offset, new_thruster.max_force);
            uncontrolled_thrusters.Add(thruster, new_thruster);
            log_ECU_action("assign_thruster", string.Format("{0} ({1}) [{2}]\n\t\t\tCentre position: {3}\n\t\t\tOffset from CoM: {4} ({5} m)\n\t\t\tMaximum torque: {6} MN*m", 
                thruster.CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId, 
                new_thruster.grid_centre_pos, 
                new_thruster.CoM_offset, new_thruster.CoM_offset.Length(), 
                new_thruster.max_torque.Length() / 1E+6));
        }

        private void dispose_thruster(MyThrust thruster)
        {
            bool thruster_found = false;
            if (uncontrolled_thrusters.ContainsKey(thruster))
            {
                thruster_found = true;
                uncontrolled_thrusters.Remove(thruster);
                log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", thruster.CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
            }
            else
            {
                foreach (var cur_direction in controlled_thrusters)
                {
                    if (cur_direction.ContainsKey(thruster))
                    {
                        thruster_found = true;
                        cur_direction.Remove(thruster);
                        log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", thruster.CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
                        break;
                    }
                }
            }
            Debug.Assert(thruster_found, "TT&DT engine_control_unit.dispose_thruster ERROR: " + thruster.CustomName + " [" + thruster.EntityId + "] hasn't been registered");
        }

        private void on_block_added(MySlimBlock block)
        {
            if (disposed)
                throw new ObjectDisposedException("ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] didn't deregister event handlers.");
            if (block.FatBlock != null)
            {
                log_ECU_action("on_block_added", block.ToString());
                var thruster = block.FatBlock as MyThrust;
                if (thruster != null)
                    assign_thruster(thruster);
            }
        }

        private void on_block_removed(MySlimBlock block)
        {
            if (disposed)
                throw new ObjectDisposedException("ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] didn't deregister event handlers.");
            if (block.FatBlock != null)
            {
                log_ECU_action("on_block_removed", block.ToString());
                var thruster = block.FatBlock as MyThrust;
                if (thruster != null)
                    dispose_thruster(thruster);
            }
        }

        private engine_control_unit()
        {
            throw new InvalidOperationException("Attempt to construct ECU without associated CubeGrid");
        }

        public engine_control_unit(MyCubeGrid grid)
        {
            Debug.Assert(grid != null, "TT&DT engine_control_unit ERROR: grid == null");
            this.grid = grid;
            grid.OnBlockAdded   += on_block_added;
            grid.OnBlockRemoved += on_block_removed;
            Type grid_systems_type = grid.GridSystems.GetType();
            PropertyInfo gyro_system_ref = grid_systems_type.GetProperty("GyroSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            gyro_control = (MyGridGyroSystem) gyro_system_ref.GetValue(grid.GridSystems);
            Debug.Assert(gyro_control != null, "TT&DT engine_control_unit ERROR: gyro_control == null");
            PropertyInfo terminal_system_ref = grid_systems_type.GetProperty("TerminalSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            grid_control = (IMyGridTerminalSystem) terminal_system_ref.GetValue(grid.GridSystems);
            Debug.Assert(grid_control != null, "TT&DT engine_control_unit ERROR: grid_control == null");
            thrust_control = grid.Components.Get<MyEntityThrustComponent>();

            inverse_world_transform = grid.PositionComp.WorldMatrixNormalizedInv;
            grid_CoM_location = Vector3D.Transform(grid.Physics.CenterOfMassWorld, inverse_world_transform);
            var thruster_list = new List<IMyTerminalBlock>();
            grid_control.GetBlocksOfType<IMyThrust>(thruster_list);
            foreach (var cur_thruster in thruster_list)
                assign_thruster((MyThrust) cur_thruster);
        }

        ~engine_control_unit()
        {
            Debug.Assert(disposed, "ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] hasn't been disposed properly.");
        }

        public void Dispose()
        {
            if (!disposed)
            {
                grid.OnBlockAdded   -= on_block_added;
                grid.OnBlockRemoved -= on_block_removed;
                disposed = true;
                log_ECU_action("Dispose", string.Format("ECU for grid {0} [{1}] has been disposed", grid.DisplayName, grid.EntityId));
            }
        }
    }
}
