using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Cube;
using Sandbox.Game.GameSystems;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game.Components;
using VRage.Utils;
using VRageMath;

namespace thruster_torque_and_differential_throttling
{
    class engine_control_unit: IDisposable
    {
        #region fields

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5 };
        class thruster_info     // Technically a struct, but C# doesnt' allow to modify fields of a struct which is implemented as property
        {
            public float      max_force;
            public Vector3    max_torque;
            public Vector3    grid_centre_pos;
            public Vector3    static_moment;
            public Vector3    CoM_offset;
            public thrust_dir nozzle_direction;
            public float      current_setting;
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
        private float[] max_force        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private float[] control_vector   = new float[6];
        private float[] braking_vector   = new float[6];
        private float[] desired_force    = new float[6];
        private  bool[] dampers_disabled = { false, false, false, false, false, false };
        private Dictionary<MyThrust, thruster_info> uncontrolled_thrusters = new Dictionary<MyThrust, thruster_info>();
        private List<MyThrust> shallow_copy = new List<MyThrust>();

        private bool      disposed = false;
        private Vector3D  grid_CoM_location;
        private MatrixD   inverse_world_transform;
        private FieldInfo max_gyro_torque_ref;
        private float     max_gyro_torque = 0.0f, max_gyro_torque_squared = 0.0f;
        private Vector3   local_angular_velocity;
        private float     speed;

        #endregion

        #region DEBUG
        [Conditional("DEBUG")]
        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT \t\tengine_control_unit<{0} [{1}]>.{2}(): {3}", grid.DisplayName, grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (var cur_direction in controlled_thrusters)
                num_controlled_thrusters += cur_direction.Count;
            MyLog.Default.WriteLine(string.Format("TT&DT \t\ttotal thrusters: {0} ({1}/{2}/{3}/{4}/{5}/{6} controlled, {7} uncontrolled)", 
                uncontrolled_thrusters.Count + num_controlled_thrusters,
                controlled_thrusters[(int) thrust_dir.fore     ].Count,
                controlled_thrusters[(int) thrust_dir.aft      ].Count,
                controlled_thrusters[(int) thrust_dir.starboard].Count,
                controlled_thrusters[(int) thrust_dir.port     ].Count,
                controlled_thrusters[(int) thrust_dir.dorsal   ].Count,
                controlled_thrusters[(int) thrust_dir.ventral  ].Count,
                uncontrolled_thrusters.Count));
        }

        [Conditional("DEBUG")]
        private void screen_text(string method_name, string message, int display_time_ms)
        {
            Sandbox.ModAPI.MyAPIGateway.Utilities.ShowNotification(string.Format("engine_control_unit.{0}(): {1}", method_name, message), display_time_ms);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info()
        {
            thruster_info cur_thruster_info;

            foreach (var cur_direction in controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    cur_thruster_info = cur_thruster.Value;
                    cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - grid_CoM_location;
                    cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
                }
            }

            foreach (var cur_thruster in uncontrolled_thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - grid_CoM_location;
                cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
            }

            screen_text("refresh_thruster_info", grid.DisplayName, 5000);
        }

        private void check_override_on_uncontrolled()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                dampers_disabled[dir_index] = false;
            }
            foreach (var cur_thruster in uncontrolled_thrusters)
            {
                if (cur_thruster.Key.ThrustOverride > cur_thruster.Value.max_force * 0.01f)
                    dampers_disabled[(int) cur_thruster.Value.nozzle_direction] = true;
            }
            screen_text("check_override_on_uncontrolled", string.Format("{0}/{1}/{2}/{3}/{4}/{5}",
                    dampers_disabled[(int) thrust_dir.fore     ],
                    dampers_disabled[(int) thrust_dir.aft      ],
                    dampers_disabled[(int) thrust_dir.starboard],
                    dampers_disabled[(int) thrust_dir.port     ],
                    dampers_disabled[(int) thrust_dir.dorsal   ],
                    dampers_disabled[(int) thrust_dir.ventral  ]
                ), 5000);
        }

        private void calculate_and_apply_torque()
        {
            Vector3 torque = Vector3.Zero;

            foreach (var cur_direction in controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    if (cur_thruster.Key.IsWorking)
                        torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
                }
            }

            foreach (var cur_thruster in uncontrolled_thrusters)
            {
                if (cur_thruster.Key.IsWorking)
                    torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
            }

            if (gyro_control.ControlTorque.LengthSquared() <= 0.0001f && Vector3.Dot(torque, local_angular_velocity) >= 0.0f)
            {
                if (torque.LengthSquared() > max_gyro_torque_squared)
                    torque -= Vector3.Normalize(torque) * max_gyro_torque;
                else
                    torque = Vector3.Zero;
            }
            grid.Physics.AddForce(MyPhysicsForceType.ADD_BODY_FORCE_AND_BODY_TORQUE, Vector3.Zero, null, torque);
        }

        #endregion

        #region thrust control

        private void decompose_vector(Vector3 source_vector, float[] decomposed_vector)
        {
            decomposed_vector[(int) thrust_dir.fore     ] = (source_vector.Z > 0.0f) ? ( source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.aft      ] = (source_vector.Z < 0.0f) ? (-source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.port     ] = (source_vector.X > 0.0f) ? ( source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.starboard] = (source_vector.X < 0.0f) ? (-source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.ventral  ] = (source_vector.Y > 0.0f) ? ( source_vector.Y) : 0.0f;
            decomposed_vector[(int) thrust_dir.dorsal   ] = (source_vector.Y < 0.0f) ? (-source_vector.Y) : 0.0f;
        }

        private void apply_thrust_settings()
        {
            const float MIN_OVERRIDE = 2.0f;
            float setting;

            foreach (var cur_direction in controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    if (!cur_thruster.Key.IsWorking)
                        continue;
                    setting = cur_thruster.Value.current_setting * 100.0f;
                    if (setting < MIN_OVERRIDE && speed > 0.1f)
                        setting = MIN_OVERRIDE;
                    cur_thruster.Key.SetValueFloat("Override", setting);
                }
            }
        }

        void adjust_thrust_for_rotation(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float DAMPING_CONSTANT = 10.0f;

            Vector3 desired_force_vector;
            float   desired_setting;

            foreach (var cur_thruster in controlled_thrusters[cur_dir])
            {
                desired_force_vector = Vector3.Cross(desired_angular_velocity - local_angular_velocity, cur_thruster.Value.CoM_offset);
                decompose_vector(desired_force_vector, desired_force);
                if (desired_force[cur_dir] > 0.0f)
                {
                    desired_setting = DAMPING_CONSTANT * grid.Physics.Mass * desired_force[cur_dir] / max_force[cur_dir];
                    cur_thruster.Value.current_setting += desired_setting;
                    if (cur_thruster.Value.current_setting > 1.0f)
                        cur_thruster.Value.current_setting = 1.0f;
                }
                else if (desired_force[opposite_dir] > 0.0f)
                {
                    desired_setting = DAMPING_CONSTANT * grid.Physics.Mass * desired_force[opposite_dir] / max_force[opposite_dir];
                    cur_thruster.Value.current_setting -= desired_setting;
                    if (cur_thruster.Value.current_setting < 0.0f)
                        cur_thruster.Value.current_setting = 0.0f;
                }
            }
        }

        private void handle_thrust_control()
        {
            const float DAMPING_CONSTANT = -2.0f;

            int     opposite_dir;
            Matrix  inverse_world_rotation   = inverse_world_transform.GetOrientation();
            Vector3 local_linear_velocity    = Vector3.Transform(grid.Physics.LinearVelocity , inverse_world_rotation);
            local_angular_velocity           = Vector3.Transform(grid.Physics.AngularVelocity, inverse_world_rotation);
            Vector3 desired_angular_velocity = gyro_control.ControlTorque * 10.0f;
            speed = local_linear_velocity.Length();

            decompose_vector(thrust_control.ControlThrust, control_vector);
            if (thrust_control.DampenersEnabled)
            {
                decompose_vector(local_linear_velocity * (grid.Physics.Mass * DAMPING_CONSTANT), braking_vector);
                opposite_dir = 3;
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    if (!dampers_disabled[dir_index] && max_force[dir_index] > 0.0f && control_vector[opposite_dir] < 0.01f)
                    {
                        control_vector[dir_index] += braking_vector[dir_index] / max_force[dir_index];
                        if (control_vector[dir_index] > 1.0f)
                            control_vector[dir_index] = 1.0f;
                    }
                    if (++opposite_dir >= 6)
                        opposite_dir = 0;
                }
            }

            opposite_dir = 3;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (var cur_thruster in controlled_thrusters[dir_index])
                    cur_thruster.Value.current_setting = control_vector[dir_index];
                if (!dampers_disabled[dir_index] && control_vector[opposite_dir] < 0.01f)
                    adjust_thrust_for_rotation(dir_index, opposite_dir, desired_angular_velocity);
                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            apply_thrust_settings();
        }

        #endregion

        #region thruster manager

        private void on_thrust_override_changed(float setting)
        {
            check_override_on_uncontrolled();
            //screen_text("on_thrust_override_changed", setting.ToString(), 5000);
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

        private void check_thruster_control_changed()
        {
            bool changes_made = false;
            int  dir_index;

            for (dir_index = 0; dir_index < 6; ++dir_index)
            {
                Dictionary<MyThrust, thruster_info> cur_direction = controlled_thrusters[dir_index];
                shallow_copy.Clear();
                shallow_copy.AddRange(cur_direction.Keys);
                foreach (var cur_thruster in shallow_copy)
                {
                    if (!cur_thruster.IsWorking || !cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
                    {
                        max_force[dir_index] -= cur_direction[cur_thruster].max_force;
                        uncontrolled_thrusters.Add(cur_thruster, cur_direction[cur_thruster]);
                        cur_thruster.ThrustOverrideChanged += on_thrust_override_changed;
                        cur_direction.Remove(cur_thruster);
                        changes_made = true;
                    }
                }
            }

            shallow_copy.Clear();
            shallow_copy.AddRange(uncontrolled_thrusters.Keys);
            foreach (var cur_thruster in shallow_copy)
            {
                if (cur_thruster.IsWorking && cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
                {
                    dir_index = (int) uncontrolled_thrusters[cur_thruster].nozzle_direction;
                    controlled_thrusters[dir_index].Add(cur_thruster, uncontrolled_thrusters[cur_thruster]);
                    cur_thruster.ThrustOverrideChanged -= on_thrust_override_changed;
                    uncontrolled_thrusters.Remove(cur_thruster);
                    max_force[dir_index] += controlled_thrusters[dir_index][cur_thruster].max_force;
                    changes_made = true;
                }
            }

            if (changes_made)
            {
                check_override_on_uncontrolled();
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    max_force[(int) thrust_dir.starboard] / 1000.0f,
                    max_force[(int) thrust_dir.port     ] / 1000.0f,
                    max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    max_force[(int) thrust_dir.ventral  ] / 1000.0f));
            }
        }

        private void assign_thruster(MyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.grid_centre_pos  = (thruster.Min + thruster.Max) * (grid.GridSize / 2.0f);
            new_thruster.max_force        = thruster.BlockDefinition.ForceMagnitude;
            new_thruster.static_moment    = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.CoM_offset       = new_thruster.grid_centre_pos - grid_CoM_location;
            new_thruster.max_torque       = Vector3.Cross(new_thruster.CoM_offset, -thruster.ThrustForwardVector * new_thruster.max_force);
            new_thruster.nozzle_direction = get_nozzle_orientation(thruster);
            uncontrolled_thrusters.Add(thruster, new_thruster);
            thruster.ThrustOverrideChanged += on_thrust_override_changed;
            log_ECU_action("assign_thruster", string.Format("{0} ({1}) [{2}]\n\t\t\tCentre position: {3}\n\t\t\tOffset from CoM: {4} ({5} m)\n\t\t\tMaximum torque: {6} MN*m", 
                thruster.CustomName, new_thruster.nozzle_direction.ToString(), thruster.EntityId, 
                new_thruster.grid_centre_pos, 
                new_thruster.CoM_offset, new_thruster.CoM_offset.Length(), 
                new_thruster.max_torque.Length() / 1E+6));
        }

        private void dispose_thruster(MyThrust thruster)
        {
            bool thruster_found = false;
            thruster.ThrustOverrideChanged -= on_thrust_override_changed;
            if (uncontrolled_thrusters.ContainsKey(thruster))
            {
                thruster_found = true;
                uncontrolled_thrusters.Remove(thruster);
                check_override_on_uncontrolled();
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

            Type gyro_system_type = gyro_control.GetType();
            max_gyro_torque_ref   = gyro_system_type.GetField("m_maxGyroForce", BindingFlags.Instance | BindingFlags.NonPublic);

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

        #endregion

        public void handle_60Hz()
        {
            if (disposed)
                throw new ObjectDisposedException("ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] is no longer functional.");
            inverse_world_transform = grid.PositionComp.WorldMatrixNormalizedInv;
            if (!grid.IsStatic && grid.Physics != null)
            {
                thrust_control = grid.Components.Get<MyEntityThrustComponent>();
                if (thrust_control != null)
                {
                    handle_thrust_control();
                    calculate_and_apply_torque();
                }
            }
        }

        public void handle_4Hz()
        {
            if (disposed)
                throw new ObjectDisposedException("ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] is no longer functional.");
            if (grid.IsStatic || grid.Physics == null || thrust_control == null)
                return;
            var current_grid_CoM = Vector3D.Transform(grid.Physics.CenterOfMassWorld, inverse_world_transform);
            if ((current_grid_CoM - grid_CoM_location).LengthSquared() > 0.01f)
            {
                grid_CoM_location = current_grid_CoM;
                refresh_thruster_info();
            }
            max_gyro_torque         = (float) max_gyro_torque_ref.GetValue(gyro_control);
            max_gyro_torque_squared = max_gyro_torque * max_gyro_torque;
            screen_text("handle_4Hz", string.Format("ROT = {0}, LIN = {1}", gyro_control.ControlTorque, thrust_control.ControlThrust), 200);
        }

        public void handle_2s_period()
        {
            if (disposed)
                throw new ObjectDisposedException("ECU for grid \"" + grid.DisplayName + "\" [" + grid.EntityId.ToString() + "] is no longer functional.");
            if (grid.IsStatic || grid.Physics == null || thrust_control == null)
                return;
            check_thruster_control_changed();
        }
    }
}
