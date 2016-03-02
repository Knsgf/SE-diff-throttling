using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Cube;
using Sandbox.Game.GameSystems;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game;
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
            public float      max_force, actual_max_force;
            public Vector3    max_torque;
            public Vector3    grid_centre_pos;
            public Vector3    static_moment;
            public Vector3    CoM_offset;
            public Vector3    reference_vector;
            //public Vector3    reference_normalised;
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
        private float[] _control_vector   = new float[6], _control_vector_copy = new float[6];
        private float[] _braking_vector   = new float[6];
        private float[] _linear_component = new float[6];
        private float[] _requested_force  = new float[6];
        private float[] _actual_force     = new float[6];
        private  bool[] _dampers_disabled = { false, false, false, false, false, false };

        private bool      _disposed = false, _gyro_override_active = false;
        private Vector3D  _grid_CoM_location;
        private MatrixD   _inverse_world_transform;
        private Matrix    _inverse_world_rotation_fixed;
        private FieldInfo _max_gyro_torque_ref, _gyro_override_ref;
        private float     _max_gyro_torque = 0.0f, _max_gyro_torque_squared = 0.0f;
        private float     _specific_moment_of_inertia;  // Grid's approximate spherical moment of inertia divided by mean radius

        private Vector3 _local_angular_velocity, _prev_angular_velocity = Vector3.Zero;
        private float   _speed;
        private bool    _current_mode_is_steady_velocity = false, _new_mode_is_steady_velocity = false;
        private Vector3 _desired_angular_velocity, _captured_angular_velocity, _rotational_integral = Vector3.Zero, _last_control_dir = Vector3.Zero;
        private bool    _enable_integral = true, _restrict_integral = false, _all_engines_off;

        private  bool   _allow_extra_linear_opposition = false, _integral_cleared = false;
        private  bool[] _enable_linear_integral    = {  true,  true,  true,  true,  true,  true };
        //private  bool[] _reset_linear_integral     = { false, false, false, false, false, false };
        private float[] _linear_integral           = {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f };
        private float[] _local_gravity_inv         = new float[6];
        private float[] _local_linear_velocity_inv = new float[6];

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
            if (method_name == "")
                Sandbox.ModAPI.MyAPIGateway.Utilities.ShowNotification(string.Format("\"{0}\" {1}", _grid.DisplayName, message), display_time_ms);
            else
                Sandbox.ModAPI.MyAPIGateway.Utilities.ShowNotification(string.Format("engine_control_unit.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message), display_time_ms);
        }

        [Conditional("DEBUG")]
        private void screen_vector<type>(string method_name, string vector_name, type[] vector, int display_time_ms)
        {
            screen_text(method_name, string.Format("{0} = {1:F3}/{2:F3}/{3:F3}/{4:F3}/{5:F3}/{6:F3}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info_for_single_direction(Dictionary<MyThrust, thruster_info> thrusters)
        {
            thruster_info cur_thruster_info;

            foreach (var cur_thruster in thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.CoM_offset = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.max_torque = Vector3.Cross(cur_thruster_info.CoM_offset, -cur_thruster.Key.ThrustForwardVector * cur_thruster.Key.BlockDefinition.ForceMagnitude);
            }
        }

        private void refresh_thruster_info()
        {
            foreach (var cur_direction in _controlled_thrusters)
                refresh_thruster_info_for_single_direction(cur_direction);
            refresh_thruster_info_for_single_direction(_uncontrolled_thrusters);
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

            //_are_gyroscopes_saturated = _max_gyro_torque < 1.0f || torque.LengthSquared() / _max_gyro_torque_squared >= 0.75f * 0.75f;
            if (_gyro_control.ControlTorque.LengthSquared() <= 0.0001f)
            {
                float gyro_limit = _gyro_control.ResourceSink.SuppliedRatio * (_max_gyro_torque - _gyro_control.Torque.Length());
                if (gyro_limit > 1.0f)
                {
                    Vector3 gyro_torque = _desired_angular_velocity - _local_angular_velocity;
                    if (gyro_torque.LengthSquared() > 1.0f)
                        gyro_torque.Normalize();
                    gyro_torque     *= gyro_limit;
                    torque          += gyro_torque;
                    _all_engines_off = false;
                }
                /*
                if (torque.LengthSquared() <= _max_gyro_torque_squared)
                    torque  = Vector3.Zero;
                else
                    torque -= Vector3.Normalize(torque) * _max_gyro_torque;
                */
            }
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
            const float MIN_OVERRIDE = 1.001f;

            float         setting;
            bool          enforce_min_override;
            thruster_info cur_thruster_info;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                enforce_min_override = _control_vector[dir_index] > 0.01f && _speed > 0.1f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                    {
                        if (cur_thruster_info.prev_setting != 0)
                        {
                            cur_thruster.Key.SetValueFloat("Override", 0.0f);
                            cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0;
                        }
                        continue;
                    }
                    setting = cur_thruster_info.current_setting * 100.0f;
                    if (enforce_min_override && setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    if ((int) setting != cur_thruster_info.prev_setting)
                    {
                        cur_thruster.Key.SetValueFloat("Override", setting);
                        cur_thruster_info.prev_setting = (int) setting;
                    }
                }
            }
        }

        private void initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f, INTEGRAL_CONSTANT = 0.05f;

            _allow_extra_linear_opposition = _thrust_control.ControlThrust.LengthSquared() > 0.75f * 0.75f;
            decompose_vector(_thrust_control.ControlThrust, _control_vector);
            if (!_thrust_control.DampenersEnabled)
            {
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        _enable_linear_integral[dir_index] = /*_reset_linear_integral[dir_index] =*/ false;
                        _linear_integral[dir_index]        = 0.0f;
                    }
                    _integral_cleared = true;
                }
            }
            else
            {
                _integral_cleared = false;
                decompose_vector((local_linear_velocity_vector * DAMPING_CONSTANT - local_gravity_vector) * _grid.Physics.Mass,
                    _braking_vector);
                decompose_vector(-local_gravity_vector        , _local_gravity_inv        );
                decompose_vector(-local_linear_velocity_vector, _local_linear_velocity_inv);
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                    _control_vector_copy[dir_index] = _control_vector[dir_index];

                int opposite_dir = 3;
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    _enable_linear_integral[dir_index] = _local_gravity_inv[dir_index] > 0.0f
                        && _control_vector_copy[dir_index] < 0.01f && _control_vector_copy[opposite_dir] < 0.01f;

                    if (_dampers_disabled[dir_index] || _controlled_thrusters[dir_index].Count == 0 || _max_force[dir_index] <= 0.0f)
                        _enable_linear_integral[dir_index] = /*_reset_linear_integral[dir_index] =*/ false;
                    else if (_control_vector_copy[opposite_dir] < 0.01f)
                    {
                        _control_vector[dir_index] += (_braking_vector[dir_index] + _grid.Physics.Mass * _linear_integral[dir_index]) 
                            / _max_force[dir_index];
                        if (_control_vector[dir_index] >= 1.0f)
                        {
                            _control_vector[dir_index]         = 1.0f;
                            _enable_linear_integral[dir_index] = /*_reset_linear_integral[dir_index] =*/ false;
                        }
                        if (_control_vector[dir_index] > 0.75f)
                            _allow_extra_linear_opposition = true;
                    }

                    /*if (_reset_linear_integral[dir_index] && _local_linear_velocity_inv[dir_index] <= 0.0f)
                    {
                        _reset_linear_integral[dir_index] = false;
                        //_linear_integral[dir_index]       = 0.0f;
                    }
                    else*/ if (_enable_linear_integral[dir_index])
                    {
                        _linear_integral[dir_index] += INTEGRAL_CONSTANT * (_local_linear_velocity_inv[dir_index] - _local_linear_velocity_inv[opposite_dir]);
                        if (_linear_integral[dir_index] < 0.0f)
                            _linear_integral[dir_index] = 0.0f;
                    }
                    else
                    {
                        //_reset_linear_integral[dir_index] = true;
                        _linear_integral[dir_index]       = 0.0f;
                    }

                    if (++opposite_dir >= 6)
                        opposite_dir = 0;
                }
            }
        }

        void adjust_thrust_for_steering(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity)
        {
            const float THRUST_INCREASE_SENSITIVITY = 0.5f, THRUST_REDUCTION_SENSITIVITY = 0.5f;
            const float MIN_LINEAR_OPPOSITION = 0.1f, MAX_LINEAR_OPPOSITION = 0.5f;

            Vector3       angular_velocity_diff = desired_angular_velocity - _local_angular_velocity;
            float         desired_setting, max_linear_opposition;
            thruster_info cur_thruster_info;

            _actual_force[cur_dir] = 0.0f;
            if (_max_force[cur_dir] <= 0.0f)
                return;
            max_linear_opposition = MIN_LINEAR_OPPOSITION * _control_vector[opposite_dir];
            if (_allow_extra_linear_opposition)
                max_linear_opposition += MAX_LINEAR_OPPOSITION * (1.0f - _control_vector[opposite_dir]);
            if (_max_force[opposite_dir] < _max_force[cur_dir])
                max_linear_opposition *= _max_force[opposite_dir] / _max_force[cur_dir];
            foreach (var cur_thruster in _controlled_thrusters[cur_dir])
            {
                if (!cur_thruster.Key.IsWorking)
                    continue;
                cur_thruster_info = cur_thruster.Value;
                if (cur_thruster_info.actual_max_force < 1.0f)
                    continue;
                decompose_vector(Vector3.Cross(angular_velocity_diff, cur_thruster_info.reference_vector), _linear_component);
                if (_linear_component[cur_dir] > 0.0f)
                {
                    desired_setting = THRUST_INCREASE_SENSITIVITY * _linear_component[cur_dir] * _specific_moment_of_inertia / _max_force[cur_dir];
                    //desired_setting = (THRUST_INCREASE_SENSITIVITY * _linear_component[cur_dir] * _grid.Physics.Mass / _max_force[cur_dir]);
                    cur_thruster_info.current_setting += desired_setting;
                    if (_control_vector[opposite_dir] > 0.01f)
                    {
                        // Limit thrusters opposing player/ID linear input
                        if (cur_thruster_info.current_setting > max_linear_opposition)
                            cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    else if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;
                }
                else if (_linear_component[opposite_dir] > 0.0f)
                {
                    desired_setting = THRUST_REDUCTION_SENSITIVITY * _linear_component[opposite_dir] * _specific_moment_of_inertia / _max_force[cur_dir];
                    //desired_setting = THRUST_REDUCTION_SENSITIVITY * _linear_component[opposite_dir] * _grid.Physics.Mass / _max_force[cur_dir];
                    cur_thruster_info.current_setting -= desired_setting;
                    if (cur_thruster_info.current_setting < 0.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else if (_control_vector[opposite_dir] > 0.01f && cur_thruster_info.current_setting > max_linear_opposition)
                    {
                        // Limit thrusters opposing player/ID linear input
                        cur_thruster_info.current_setting = max_linear_opposition;
                    }
                }

                _actual_force[cur_dir] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
            }
        }

        // Ensures that resulting linear force doesn't exceed player/ID input (to prevent undesired drift when turning)
        void normalise_thrust()
        {
            int   opposite_dir = 3;
            float new_force_ratio;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (_actual_force[dir_index] >= 1.0f && _actual_force[dir_index] - _requested_force[dir_index] > _actual_force[opposite_dir])
                {
                    new_force_ratio = (_actual_force[opposite_dir] + _requested_force[dir_index]) / _actual_force[dir_index];
                    foreach (var cur_thruster in _controlled_thrusters[dir_index])
                        cur_thruster.Value.current_setting *= new_force_ratio;
                }
                if (_actual_force[opposite_dir] >= 1.0f && _actual_force[opposite_dir] - _requested_force[opposite_dir] > _actual_force[dir_index])
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
            const float ANGULAR_INTEGRAL_COEFF = -0.1f, ANGULAR_DERIVATIVE_COEFF = -0.01f;

            int opposite_dir;

            // Using a "fixed" (it changes orientation only when the player steers a ship) inverse rotation matrix here to 
            // prevent Dutch Roll-like tendencies at high speeds
            Vector3 local_linear_velocity = Vector3.Transform(_grid.Physics.LinearVelocity, _inverse_world_rotation_fixed);

            Matrix inverse_world_rotation      = _inverse_world_transform.GetOrientation();
            Vector3 local_gravity              = Vector3.Transform(_grid.Physics.Gravity            , inverse_world_rotation);
            _local_angular_velocity            = Vector3.Transform(_grid.Physics.AngularVelocity    , inverse_world_rotation);
            Vector3 local_angular_acceleration = (_local_angular_velocity - _prev_angular_velocity) / MyEngineConstants.UPDATE_STEP_SIZE_IN_SECONDS;
            _prev_angular_velocity             = _local_angular_velocity;
            _speed = local_linear_velocity.Length();

            initialise_linear_controls(local_linear_velocity, local_gravity);

            if (_gyro_control.ControlTorque.LengthSquared() > 0.0001f)
            {
                _last_control_dir    = _gyro_control.ControlTorque;
                _rotational_integral = (_rotational_integral + ANGULAR_INTEGRAL_COEFF * _local_angular_velocity) 
                    * Vector3.IsZeroVector(_last_control_dir, 0.01f);
                _desired_angular_velocity     = _gyro_control.ControlTorque * 15.0f + _rotational_integral + ANGULAR_DERIVATIVE_COEFF * local_angular_acceleration;
                _enable_integral              = _restrict_integral = false;
                _inverse_world_rotation_fixed = inverse_world_rotation;
            }
            else
            {
                if (_restrict_integral && Vector3.Dot(_captured_angular_velocity, _local_angular_velocity) <= 0.0f)
                {
                    _captured_angular_velocity    = Vector3.Zero;
                    _restrict_integral            = false;
                    _inverse_world_rotation_fixed = inverse_world_rotation;
                }
                else if (_enable_integral)
                {
                    if (_all_engines_off)
                        _rotational_integral = Vector3.Zero;
                    else
                    {
                        Vector3 rotational_integral_change = _local_angular_velocity * ANGULAR_INTEGRAL_COEFF;
                        if (_restrict_integral && Vector3.Dot(_captured_angular_velocity, local_angular_acceleration) < 0.0f /*&& local_angular_acceleration.LengthSquared() > 0.0003f*/)
                            rotational_integral_change *= Vector3.IsZeroVector(_last_control_dir, 0.01f);
                        if (_rotational_integral.LengthSquared() < 10.0f || Vector3.Dot(_rotational_integral, rotational_integral_change) < 0.0f)
                            _rotational_integral += rotational_integral_change;
                    }

                    // Force update of "fixed" inverse rotation matrix when angle exceeds 11 degrees
                    if (Vector3.Dot(_inverse_world_rotation_fixed.Forward, inverse_world_rotation.Forward) < 0.98f)
                        _inverse_world_rotation_fixed = inverse_world_rotation;
                }
                else
                {
                    _captured_angular_velocity    = _local_angular_velocity;
                    _restrict_integral            = _enable_integral = true;
                    _inverse_world_rotation_fixed = inverse_world_rotation;
                }
                _desired_angular_velocity = -_local_angular_velocity + _rotational_integral 
                    + ANGULAR_DERIVATIVE_COEFF * local_angular_acceleration;
            }
            //screen_text("", String.Format("DAV = {0}, RI = {1}, LAC = {2}", _desired_angular_velocity.Length(), _restrict_integral, Vector3.Dot(_captured_angular_velocity, local_angular_acceleration)), 16);

            _new_mode_is_steady_velocity    = _all_engines_off = true;
            _allow_extra_linear_opposition |= _gyro_control.ControlTorque.LengthSquared() > 0.0001f || _local_angular_velocity.LengthSquared() > 0.0003f;
            opposite_dir                    = 3;
            thruster_info cur_thruster_info;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (_control_vector[dir_index] > 0.01f)
                    _new_mode_is_steady_velocity = false;
                _requested_force[dir_index] = 0.0f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    _all_engines_off  = false;
                    cur_thruster_info = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else
                    {
                        cur_thruster_info.current_setting = _control_vector[dir_index];
                        _requested_force[dir_index]      += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                }
                if (_gyro_override_active || _dampers_disabled[dir_index])
                    _actual_force[dir_index] = _requested_force[dir_index];
                else
                    adjust_thrust_for_steering(dir_index, opposite_dir, _desired_angular_velocity);
                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            normalise_thrust();
            apply_thrust_settings();
            // Ensure that steady-velocity mode won't cause integral to wind up out of control
            _new_mode_is_steady_velocity &= _rotational_integral.LengthSquared() < 0.01f;
        }

        private static void set_thruster_reference_vector(thruster_info thruster, Vector3 reference)
        {
            thruster.reference_vector     = reference;
            //thruster.reference_normalised = (reference.LengthSquared() > 0.01f) ? Vector3.Normalize(reference) : Vector3.Zero;
        }

        private void update_reference_vectors_for_accelerating_mode()
        {
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                    set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
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
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.CoM_offset);
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
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.grid_centre_pos - CoT_location);
                    foreach (var cur_thruster in _controlled_thrusters[dir_index + 3])
                        set_thruster_reference_vector(cur_thruster.Value, cur_thruster.Value.grid_centre_pos - CoT_location);
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
                    if (   !cur_thruster.IsWorking 
                        || cur_direction[cur_thruster].actual_max_force < 0.01f * cur_direction[cur_thruster].max_force 
                        || !cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
                    {
                        cur_thruster.SetValueFloat("Override", 0.0f);
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
                if (   cur_thruster.IsWorking 
                    && _uncontrolled_thrusters[cur_thruster].actual_max_force > 0.01f * _uncontrolled_thrusters[cur_thruster].max_force 
                    && cur_thruster.CustomName.ToString().ToUpper().Contains("[RCS]"))
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

        private void refresh_real_max_forces()
        {
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                    cur_thruster.Value.actual_max_force = cur_thruster.Value.max_force * _thrust_control.GetLastThrustMultiplier(cur_thruster.Key);
            }
            foreach (var cur_thruster in _uncontrolled_thrusters)
                cur_thruster.Value.actual_max_force = cur_thruster.Value.max_force * _thrust_control.GetLastThrustMultiplier(cur_thruster.Key);
        }

        private void assign_thruster(MyThrust thruster)
        {
            var new_thruster = new thruster_info();
            new_thruster.grid_centre_pos  = (thruster.Min + thruster.Max) * (_grid.GridSize / 2.0f);
            new_thruster.max_force        = new_thruster.actual_max_force = thruster.BlockDefinition.ForceMagnitude;
            new_thruster.static_moment    = new_thruster.grid_centre_pos * new_thruster.max_force;
            new_thruster.CoM_offset       = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.max_torque       = Vector3.Cross(new_thruster.CoM_offset, -thruster.ThrustForwardVector * new_thruster.max_force);
            new_thruster.nozzle_direction = get_nozzle_orientation(thruster);
            set_thruster_reference_vector(new_thruster, new_thruster.CoM_offset);
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

        private void calc_spherical_moment_of_inertia()
        {
            Vector3 grid_dim            = (_grid.Max - _grid.Min) * _grid.GridSize;
            float grid_volume           = Math.Abs(grid_dim.X * grid_dim.Y * grid_dim.Z);
            float reference_radius      = (float) Math.Pow(grid_volume / (4.0 / 3.0 * Math.PI), 1.0 / 3.0);
            _specific_moment_of_inertia = 0.4f * _grid.Physics.Mass * reference_radius;
            log_ECU_action("calc_spherical_moment_of_inertia", string.Format("volume = {0} m3, radius = {1} m, SMoI = {2} t*m", grid_volume, reference_radius, _specific_moment_of_inertia / 1000.0f));
        }

        private void on_block_added(MySlimBlock block)
        {
            if (_disposed)
                throw new ObjectDisposedException("ECU for grid \"" + _grid.DisplayName + "\" [" + _grid.EntityId.ToString() + "] didn't deregister event handlers.");
            calc_spherical_moment_of_inertia();
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
            calc_spherical_moment_of_inertia();
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
            _grid = grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            _inverse_world_transform      = _grid.PositionComp.WorldMatrixNormalizedInv;
            _grid_CoM_location            = Vector3D.Transform(_grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            _inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();
            calc_spherical_moment_of_inertia();

            Type grid_systems_type = _grid.GridSystems.GetType();
            PropertyInfo gyro_system_ref = grid_systems_type.GetProperty("GyroSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            _gyro_control = (MyGridGyroSystem) gyro_system_ref.GetValue(_grid.GridSystems);
            Debug.Assert(_gyro_control != null, "TT&DT engine_control_unit ERROR: gyro_control == null");
            Type gyro_system_type = _gyro_control.GetType();
            _max_gyro_torque_ref  = gyro_system_type.GetField("m_maxGyroForce"    , BindingFlags.Instance | BindingFlags.NonPublic);
            _gyro_override_ref    = gyro_system_type.GetField("m_maxOverrideForce", BindingFlags.Instance | BindingFlags.NonPublic);

            PropertyInfo terminal_system_ref = grid_systems_type.GetProperty("TerminalSystem", BindingFlags.Instance | BindingFlags.NonPublic);
            var grid_control = (IMyGridTerminalSystem)terminal_system_ref.GetValue(_grid.GridSystems);
            Debug.Assert(grid_control != null, "TT&DT engine_control_unit ERROR: grid_control == null");
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
                foreach (var cur_thruster in _uncontrolled_thrusters.Keys)
                    cur_thruster.ThrustOverrideChanged -= on_thrust_override_changed;
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
            refresh_real_max_forces();
            _gyro_override_active    = (float)   _gyro_override_ref.GetValue(_gyro_control) > 0.01f;
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
