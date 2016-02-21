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
            public Vector3    reference_vector;
            public thrust_dir nozzle_direction;
            public float      current_setting;
            public int        prev_setting;
        };

        private MyCubeGrid              _grid;
        private MyGridGyroSystem        _gyro_control;
        private MyEntityThrustComponent _thrust_control;
        private Dictionary<MyThrust, thruster_info>[] _controlled_thrusters =
        {
            new Dictionary<MyThrust, thruster_info>(),   // fore
            new Dictionary<MyThrust, thruster_info>(),   // starboard
            new Dictionary<MyThrust, thruster_info>(),   // dorsal
            new Dictionary<MyThrust, thruster_info>(),   // aft
            new Dictionary<MyThrust, thruster_info>(),   // port
            new Dictionary<MyThrust, thruster_info>()    // ventral
        };
        private Dictionary<MyThrust, thruster_info> _uncontrolled_thrusters = new Dictionary<MyThrust, thruster_info>();
        private List<MyThrust> _shallow_copy = new List<MyThrust>();
        private float[] _max_force        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private float[] _control_vector   = new float[6];
        private float[] _braking_vector   = new float[6];
        private float[] _desired_force    = new float[6];
        private float[] _requested_force  = new float[6];
        private float[] _actual_force     = new float[6];
        private  bool[] _dampers_disabled = { false, false, false, false, false, false };

        private bool      _disposed = false;
        private Vector3D  _grid_CoM_location;
        private MatrixD   _inverse_world_transform;
        private FieldInfo _max_gyro_torque_ref;
        private float     _max_gyro_torque = 0.0f, _max_gyro_torque_squared = 0.0f;

        private Vector3 _local_angular_velocity;
        private float   _speed;
        private bool    _current_mode_is_steady_velocity = false, _new_mode_is_steady_velocity = false;
        private Vector3 _desired_angular_velocity, _captured_angular_velocity;
        private bool    _enable_integral = false, _reset_integral = false, _are_gyroscopes_saturated = false;

        private Vector3 _linear_integral = Vector3.Zero, _captured_linear_velocity = Vector3.Zero;
        private bool    _enable_linear_integral = false, _reset_linear_integral = false;

        #endregion

        #region DEBUG
        [Conditional("DEBUG")]
        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT \t\tengine_control_unit<{0} [{1}]>.{2}(): {3}", _grid.DisplayName, _grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (var cur_direction in _controlled_thrusters)
                num_controlled_thrusters += cur_direction.Count;
            MyLog.Default.WriteLine(string.Format("TT&DT \t\ttotal thrusters: {0} ({1}/{2}/{3}/{4}/{5}/{6} controlled, {7} uncontrolled)", 
                _uncontrolled_thrusters.Count + num_controlled_thrusters,
                _controlled_thrusters[(int) thrust_dir.fore     ].Count,
                _controlled_thrusters[(int) thrust_dir.aft      ].Count,
                _controlled_thrusters[(int) thrust_dir.starboard].Count,
                _controlled_thrusters[(int) thrust_dir.port     ].Count,
                _controlled_thrusters[(int) thrust_dir.dorsal   ].Count,
                _controlled_thrusters[(int) thrust_dir.ventral  ].Count,
                _uncontrolled_thrusters.Count));
        }

        [Conditional("DEBUG")]
        private void screen_text(string method_name, string message, int display_time_ms)
        {
            Sandbox.ModAPI.MyAPIGateway.Utilities.ShowNotification(string.Format("engine_control_unit.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message), display_time_ms);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info()
        {
            thruster_info cur_thruster_info;

            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    cur_thruster_info = cur_thruster.Value;
                    cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                    cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
                }
            }

            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
            }
        }

        private void check_override_on_uncontrolled()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                _dampers_disabled[dir_index] = false;
            }
            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                if (cur_thruster.Key.ThrustOverride > cur_thruster.Value.max_force * 0.01f)
                    _dampers_disabled[(int) cur_thruster.Value.nozzle_direction] = true;
            }
        }

        private void calculate_and_apply_torque()
        {
            Vector3 torque = Vector3.Zero;

            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    if (cur_thruster.Key.IsWorking)
                        torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
                }
            }

            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                if (cur_thruster.Key.IsWorking)
                    torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
            }

            _are_gyroscopes_saturated = torque.LengthSquared() / _max_gyro_torque_squared > 0.75f * 0.75f;
            if (_gyro_control.ControlTorque.LengthSquared() <= 0.0001f && Vector3.Dot(torque, _local_angular_velocity) >= 0.0f)
            {
                if (torque.LengthSquared() <= _max_gyro_torque_squared)
                    torque  = Vector3.Zero;
                else
                    torque -= Vector3.Normalize(torque) * _max_gyro_torque;
            }
            //if (_grid.Physics.IsWelded)
            //    screen_text("calculate_and_apply_torque", "is welded", 20);
            torque = Vector3.Transform(torque, _grid.WorldMatrix.GetOrientation());
            _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3.Zero, null, torque);
        }

        #endregion

        #region thrust control

        private static void decompose_vector(Vector3 source_vector, float[] decomposed_vector)
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
            bool  pre_condition;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                pre_condition = _thrust_control.DampenersEnabled && _control_vector[dir_index] > 0.01f && _speed > 0.1f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    if (!cur_thruster.Key.IsWorking)
                        continue;
                    setting = cur_thruster.Value.current_setting * 100.0f;
                    if (pre_condition && setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    if ((int) setting != cur_thruster.Value.prev_setting)
                    {
                        cur_thruster.Key.SetValueFloat("Override", setting);
                        cur_thruster.Value.prev_setting = (int) setting;
                    }
                }
            }
        }

        private void initialise_linear_controls(Vector3 local_linear_velocity, Vector3 local_gravity)
        {
            const float DAMPING_CONSTANT = -2.0f, INTEGRAL_CONSTANT = -0.05f;

            decompose_vector(_thrust_control.ControlThrust, _control_vector);
            if (!_thrust_control.DampenersEnabled)
            {
                _enable_linear_integral = _reset_linear_integral = false;
                _linear_integral        = Vector3.Zero;
            }
            else
            {
                if (!_enable_linear_integral)
                {
                    _reset_linear_integral    = true;
                    _captured_linear_velocity = local_linear_velocity;
                }
                _enable_linear_integral = _thrust_control.ControlThrust.LengthSquared() < 0.0001f;

                decompose_vector((local_linear_velocity * DAMPING_CONSTANT - local_gravity + _linear_integral) * _grid.Physics.Mass, _braking_vector);
                int opposite_dir = 3;
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    if (!_dampers_disabled[dir_index] && _max_force[dir_index] > 0.0f && _control_vector[opposite_dir] < 0.01f)
                    {
                        _control_vector[dir_index] += _braking_vector[dir_index] / _max_force[dir_index];
                        if (_control_vector[dir_index] > 1.0f)
                        {
                            _control_vector[dir_index] = 1.0f;
                            _enable_linear_integral    = _reset_linear_integral = false;
                            _linear_integral           = Vector3.Zero;
                        }
                    }
                    if (++opposite_dir >= 6)
                        opposite_dir = 0;
                }

                if (_reset_linear_integral && Vector3.Dot(local_linear_velocity, _captured_linear_velocity) <= 0.0f)
                {
                    _reset_linear_integral    = false;
                    _captured_linear_velocity = _linear_integral = Vector3.Zero;
                }
                else if (_enable_linear_integral)
                    _linear_integral += INTEGRAL_CONSTANT * local_linear_velocity;
            }
            //screen_text("initialise_linear_controls", string.Format("linear integral {0}, correction = {1}", _enable_linear_integral ? "on" : "off", _linear_integral.Length()), 20);
        }

        void adjust_thrust_for_rotation(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float DAMPING_CONSTANT = 10.0f;

            Vector3 desired_force_vector;
            float   desired_setting;

            _actual_force[cur_dir] = 0.0f;
            if (_max_force[cur_dir] < 0.001f)
                return;
            foreach (var cur_thruster in _controlled_thrusters[cur_dir])
            {
                desired_force_vector = Vector3.Cross(desired_angular_velocity - _local_angular_velocity, cur_thruster.Value.reference_vector);
                decompose_vector(desired_force_vector, _desired_force);
                if (_desired_force[cur_dir] > 0.0f)
                {
                    desired_setting = DAMPING_CONSTANT * _grid.Physics.Mass * _desired_force[cur_dir] / _max_force[cur_dir];
                    cur_thruster.Value.current_setting += desired_setting;
                    if (cur_thruster.Value.current_setting > 1.0f)
                        cur_thruster.Value.current_setting = 1.0f;
                }
                else if (_desired_force[opposite_dir] > 0.0f)
                {
                    desired_setting = DAMPING_CONSTANT * _grid.Physics.Mass * _desired_force[opposite_dir] / _max_force[cur_dir];
                    cur_thruster.Value.current_setting -= desired_setting;
                    if (cur_thruster.Value.current_setting < 0.0f)
                        cur_thruster.Value.current_setting = 0.0f;
                }

                _actual_force[cur_dir] += cur_thruster.Value.current_setting * cur_thruster.Key.ThrustForce.Length();
            }
        }

        void normalise_thrust(Vector3 requested_force_vector)
        {
            int   opposite_dir = 3;
            float new_force_ratio = 1.0f;

            decompose_vector(requested_force_vector, _requested_force);
            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (_actual_force[dir_index] - _requested_force[dir_index] > _actual_force[opposite_dir] + 0.001f)
                {
                    new_force_ratio = (_actual_force[opposite_dir] + _requested_force[dir_index]) / _actual_force[dir_index];
                    foreach (var cur_thruster in _controlled_thrusters[dir_index])
                        cur_thruster.Value.current_setting *= new_force_ratio;
                }
                if (_actual_force[opposite_dir] - _requested_force[opposite_dir] > _actual_force[dir_index] + 0.001f)
                {
                    new_force_ratio = (_actual_force[dir_index] + _requested_force[opposite_dir]) / _actual_force[opposite_dir];
                    foreach (var cur_thruster in _controlled_thrusters[opposite_dir])
                        cur_thruster.Value.current_setting *= new_force_ratio;
                }
                ++opposite_dir;
            }
        }

        private void handle_thrust_control()
        {
            const float MIN_ACCELERATION_MODE_ACC = 0.2f, ANGULAR_INTEGRAL_COEFF = -0.05f;

            int opposite_dir;
            Matrix  inverse_world_rotation   = _inverse_world_transform.GetOrientation();
            Vector3 local_linear_velocity    = Vector3.Transform(_grid.Physics.LinearVelocity , inverse_world_rotation);
            Vector3 local_gravity            = Vector3.Transform(_grid.Physics.Gravity        , inverse_world_rotation);
            _local_angular_velocity          = Vector3.Transform(_grid.Physics.AngularVelocity, inverse_world_rotation);
            Vector3 requested_force_vector   = Vector3.Zero;
            _speed = local_linear_velocity.Length();

            initialise_linear_controls(local_linear_velocity, local_gravity);

            if (_gyro_control.ControlTorque.LengthSquared() > 0.0001f)
            {
                _desired_angular_velocity = _gyro_control.ControlTorque * 15.0f;
                _enable_integral          = _reset_integral = false;
            }
            else
            {
                if (_reset_integral && Vector3.Dot(_captured_angular_velocity, _local_angular_velocity) <= 0.0f)
                {
                    _desired_angular_velocity = _captured_angular_velocity = Vector3.Zero;
                    _reset_integral = false;
                }
                else if (_enable_integral)
                {
                    if (_are_gyroscopes_saturated)
                        _desired_angular_velocity += _local_angular_velocity * ANGULAR_INTEGRAL_COEFF;
                    else
                        _desired_angular_velocity *= 0.9f;
                }
                else
                {
                    _desired_angular_velocity  = Vector3.Zero;
                    _captured_angular_velocity = _local_angular_velocity;
                    _reset_integral            = _enable_integral = true;
                }
            }

            opposite_dir = 3;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster.Value.current_setting = _control_vector[dir_index];
                    requested_force_vector += _control_vector[dir_index] * cur_thruster.Key.ThrustForce;
                }
                if (!_dampers_disabled[dir_index] && _max_force[dir_index] > 0.0f && _control_vector[opposite_dir] < 0.01f)
                    adjust_thrust_for_rotation(dir_index, opposite_dir, _desired_angular_velocity);
                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            normalise_thrust(requested_force_vector);
            apply_thrust_settings();
            _new_mode_is_steady_velocity = _grid.Physics.LinearAcceleration.LengthSquared() < MIN_ACCELERATION_MODE_ACC * MIN_ACCELERATION_MODE_ACC;
        }

        private void update_reference_vectors_for_accelerating_mode()
        {
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                    cur_thruster.Value.reference_vector = cur_thruster.Value.CoM_offset;
            }
        }

        private void update_reference_vectors_for_steady_velocity_mode()
        {
            Vector3 total_static_moment, CoT_location;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (_max_force[dir_index] < 1.0f || _max_force[dir_index + 3] < 1.0f)
                {
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ])
                        cur_thruster.Value.reference_vector = cur_thruster.Value.CoM_offset;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        cur_thruster.Value.reference_vector = cur_thruster.Value.CoM_offset;
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ].Values)
                        total_static_moment += cur_thruster.static_moment;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3].Values)
                        total_static_moment += cur_thruster.static_moment;
                    CoT_location = total_static_moment / (_max_force[dir_index] + _max_force[dir_index + 3]);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index    ])
                        cur_thruster.Value.reference_vector = cur_thruster.Value.grid_centre_pos - CoT_location;
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        cur_thruster.Value.reference_vector = cur_thruster.Value.grid_centre_pos - CoT_location;
                }
            }
        }

        #endregion

        #region thruster manager

        private void on_thrust_override_changed(float setting)
        {
            check_override_on_uncontrolled();
        }

        private static thrust_dir get_nozzle_orientation(MyThrust thruster)
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
                Dictionary<MyThrust, thruster_info> cur_direction = _controlled_thrusters[dir_index];
                _shallow_copy.Clear();
                _shallow_copy.AddRange(cur_direction.Keys);
                foreach (var cur_thruster in _shallow_copy)
                {
                    if (!cur_thruster.IsWorking || !cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
                    {
                        _max_force[dir_index] -= cur_direction[cur_thruster].max_force;
                        _uncontrolled_thrusters.Add(cur_thruster, cur_direction[cur_thruster]);
                        cur_thruster.ThrustOverrideChanged += on_thrust_override_changed;
                        cur_direction.Remove(cur_thruster);
                        changes_made = true;
                    }
                }
            }

            _shallow_copy.Clear();
            _shallow_copy.AddRange(_uncontrolled_thrusters.Keys);
            foreach (var cur_thruster in _shallow_copy)
            {
                if (cur_thruster.IsWorking && cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
                {
                    dir_index = (int) _uncontrolled_thrusters[cur_thruster].nozzle_direction;
                    _controlled_thrusters[dir_index].Add(cur_thruster, _uncontrolled_thrusters[cur_thruster]);
                    cur_thruster.ThrustOverrideChanged -= on_thrust_override_changed;
                    _uncontrolled_thrusters.Remove(cur_thruster);
                    _max_force[dir_index] += _controlled_thrusters[dir_index][cur_thruster].max_force;
                    changes_made = true;
                }
            }

            if (changes_made)
            {
                if (_current_mode_is_steady_velocity)
                    update_reference_vectors_for_steady_velocity_mode();
                else
                    update_reference_vectors_for_accelerating_mode();
                check_override_on_uncontrolled();
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
            }
        }

        private void assign_thruster(MyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.grid_centre_pos  = (thruster.Min + thruster.Max) * (_grid.GridSize / 2.0f);
            new_thruster.max_force        = thruster.BlockDefinition.ForceMagnitude;
            new_thruster.static_moment    = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.CoM_offset       = new_thruster.reference_vector = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.max_torque       = Vector3.Cross(new_thruster.CoM_offset, -thruster.ThrustForwardVector * new_thruster.max_force);
            new_thruster.nozzle_direction = get_nozzle_orientation(thruster);
            _uncontrolled_thrusters.Add(thruster, new_thruster);
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
            if (_uncontrolled_thrusters.ContainsKey(thruster))
            {
                thruster_found = true;
                _uncontrolled_thrusters.Remove(thruster);
                check_override_on_uncontrolled();
                log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", thruster.CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
            }
            else
            {
                foreach (var cur_direction in _controlled_thrusters)
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
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] didn't deregister event handlers.");
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
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] didn't deregister event handlers.");
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
            this._grid = grid;
            grid.OnBlockAdded   += on_block_added;
            grid.OnBlockRemoved += on_block_removed;

            Type grid_systems_type = grid.GridSystems.GetType();
            PropertyInfo gyro_system_ref = grid_systems_type.GetProperty("GyroSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            _gyro_control = (MyGridGyroSystem) gyro_system_ref.GetValue(grid.GridSystems);
            Debug.Assert(_gyro_control != null, "TT&DT engine_control_unit ERROR: gyro_control == null");
            PropertyInfo terminal_system_ref = grid_systems_type.GetProperty("TerminalSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            var grid_control = (IMyGridTerminalSystem) terminal_system_ref.GetValue(grid.GridSystems);
            Debug.Assert(grid_control != null, "TT&DT engine_control_unit ERROR: grid_control == null");

            Type gyro_system_type = _gyro_control.GetType();
            _max_gyro_torque_ref  = gyro_system_type.GetField("m_maxGyroForce", BindingFlags.Instance | BindingFlags.NonPublic);

            _inverse_world_transform = grid.PositionComp.WorldMatrixNormalizedInv;
            _grid_CoM_location = Vector3D.Transform(grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            var thruster_list  = new List<IMyTerminalBlock>();
            grid_control.GetBlocksOfType<IMyThrust>(thruster_list);
            foreach (var cur_thruster in thruster_list)
                assign_thruster((MyThrust) cur_thruster);
        }

#if DEBUG
        ~engine_control_unit()
        {
            Debug.Assert(_disposed, "ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] hasn't been disposed properly.");
        }
#endif

        public void Dispose()
        {
            if (!_disposed)
            {
                _grid.OnBlockAdded   -= on_block_added;
                _grid.OnBlockRemoved -= on_block_removed;
                _disposed = true;
#if DEBUG
                GC.SuppressFinalize(this);
#endif
                log_ECU_action("Dispose", string.Format("ECU for grid {0} [{1}] has been disposed", _grid.DisplayName, _grid.EntityId));
            }
        }

        #endregion

        public void handle_60Hz()
        {
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] is no longer functional.");
            _inverse_world_transform = _grid.PositionComp.WorldMatrixNormalizedInv;
            _thrust_control          = _grid.Components.Get<MyEntityThrustComponent>();
            if (!_grid.IsStatic && _grid.Physics != null && _thrust_control != null)
            {
                handle_thrust_control();
                calculate_and_apply_torque();
            }
        }

        public void handle_4Hz()
        {
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] is no longer functional.");
            if (_grid.IsStatic || _grid.Physics == null || _thrust_control == null)
                return;
            var current_grid_CoM = Vector3D.Transform(_grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            if ((current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f)
            {
                _grid_CoM_location = current_grid_CoM;
                refresh_thruster_info();
            }
            if (_current_mode_is_steady_velocity != _new_mode_is_steady_velocity)
            {
                if (_new_mode_is_steady_velocity)
                    update_reference_vectors_for_steady_velocity_mode();
                else
                    update_reference_vectors_for_accelerating_mode();
                _current_mode_is_steady_velocity = _new_mode_is_steady_velocity;
            }
            _max_gyro_torque         = (float) _max_gyro_torque_ref.GetValue(_gyro_control);
            _max_gyro_torque_squared = _max_gyro_torque * _max_gyro_torque;
        }

        public void handle_2s_period()
        {
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] is no longer functional.");
            if (_grid.IsStatic || _grid.Physics == null || _thrust_control == null)
                return;
            check_thruster_control_changed();
        }
    }
}
