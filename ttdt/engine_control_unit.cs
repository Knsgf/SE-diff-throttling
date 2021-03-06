﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using System.Text;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Cube;
using Sandbox.Game.GameSystems;
using Sandbox.Game.Gui;
using Sandbox.Game.World;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game;
using VRage.Game.Components;
using VRage.Utils;
using VRageMath;

namespace thruster_torque_and_differential_throttling
{
    static class StringBuilderExtensions
    {
        private static readonly string m_RCSTag  = "[RCS]";
        private static readonly string m_STATTag = "[STAT]";
        private static readonly int[] m_RCSShiftTable, m_STATShiftTable;

        #region Private methods

        private static int[] GenerateShiftTable(string tag)
        {
            int[] shiftTable = new int[char.MaxValue];

            for (int index = 0; index < char.MaxValue; ++index)
                shiftTable[index] = tag.Length;
            for (int index = 0; index < tag.Length - 1; ++index)
                shiftTable[tag[index]] = tag.Length - index - 1;
            return shiftTable;
        }

        private static bool ContainsTag(StringBuilder source, string tag, int[] tagShiftTable)
        {
            int  sourceLength = source.Length, startIndex = 0, endIndex = tag.Length - 1, tagIndex, shift;
            bool matchFound;

            while (endIndex < sourceLength)
            {
                matchFound = true;
                tagIndex   = 0;
                for (int index = startIndex; index <= endIndex; ++index)
                {
                    if (tag[tagIndex++] != source[index])
                    {
                        matchFound = false;
                        break;
                    }
                }
                if (matchFound)
                    return true;
                shift       = tagShiftTable[source[endIndex]];
                startIndex += shift;
                endIndex   += shift;
            }
            return false;
        }

        static StringBuilderExtensions()
        {
            m_RCSShiftTable  = GenerateShiftTable( m_RCSTag);
            m_STATShiftTable = GenerateShiftTable(m_STATTag);
        }

        #endregion

        public static bool ContainsRCSTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_RCSTag, m_RCSShiftTable);
        }

        public static bool ContainsSTATTag(this StringBuilder blockName)
        {
            return ContainsTag(blockName, m_STATTag, m_STATShiftTable);
        }

        public static void ToUpperTo(this StringBuilder source, StringBuilder destination)
        {
            int sourceLength = source.Length;

            destination.Length = sourceLength;
            for (int index = 0; index < sourceLength; ++index)
                destination[index] = char.ToUpper(source[index]);
        }
    }

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
            public thrust_dir nozzle_direction;
            public float      current_setting, thrust_limit;
            public int        prev_setting;
            public bool       enable_limit;
        };

        private static MyShipController       __current_controller    = null;
        private static StringBuilder          __thruster_name         = new StringBuilder();
        private static simplex_solver         __linear_solver         = new simplex_solver();
        private static MyHudNotificationDebug __control_limit_message = new MyHudNotificationDebug("WARNING: Control limit reached", 0, MyFontEnum.Red);

        private static List<     MyThrust> __thrusters_copy = new List<     MyThrust>();
        private static List<thruster_info> __thruster_infos = new List<thruster_info>();

        private static float[] __control_vector   = new float[6], __control_vector_copy = new float[6];
        private static float[] __braking_vector   = new float[6];
        private static float[] __linear_component = new float[6];
        private static float[] __requested_force  = new float[6];
        private static float[] __actual_force     = new float[6];

        private static float[] __steering_input       = new float[6];
        private static float[] __steering_output      = new float[6];
        private static float[] __angular_velocity     = new float[6];
        private static float[] __angular_acceleration = new float[6];
        private static float[] __nominal_acceleration = new float[6];

        private static float[] __local_gravity_inv         = new float[6];
        private static float[] __local_linear_velocity_inv = new float[6];

        private MyHudNotificationDebug  _thrust_reduction_message = new MyHudNotificationDebug("Thrust reduction: {0} %", 0);
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
        //private List<MyThrust> _shallow_copy = new List<MyThrust>();
        private float[] _max_force        = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        //private float[] _control_vector   = new float[6], _control_vector_copy = new float[6];
        //private float[] _braking_vector   = new float[6];
        //private float[] _linear_component = new float[6];
        //private float[] _requested_force  = new float[6];
        //private float[] _actual_force     = new float[6];
        private  bool[] _dampers_disabled = { false, false, false, false, false, false };

        private bool      _disposed = false, _gyro_override_active = false, _status_displayed = false, _RC_status_displayed = false, _calibration_scheduled = true;
        private Vector3D  _grid_CoM_location;
        private MatrixD   _inverse_world_transform;
        private Matrix    _inverse_world_rotation_fixed;
        private FieldInfo _max_gyro_torque_ref, _gyro_override_ref;
        private float     _max_gyro_torque = 0.0f, _max_gyro_torque_squared = 0.0f, _spherical_moment_of_inertia;
        private int       _thrust_reduction;

        private  bool[] _enable_integral    = { false, false, false, false, false, false };
        private  bool[] _restrict_integral  = { false, false, false, false, false, false };
        private  bool[] _calibration_used   = { false, false, false, false, false, false };
        private  bool[] _calibration_needed = { false, false, false, false, false, false };
        //private float[] _steering_input       = new float[6];
        //private float[] _steering_output      = new float[6];
        //private float[] _angular_velocity     = new float[6];
        //private float[] _angular_acceleration = new float[6];
        //private float[] _nominal_acceleration = new float[6];
        private float[] _current_trim         = new float[6];
        private float[] _last_trim            = new float[6];
        private Vector3 _local_angular_velocity, _prev_angular_velocity = Vector3.Zero, _torque, _manual_thrust, _manual_rotation;
        private float   _speed;
        private bool    _current_mode_is_steady_velocity = false, _new_mode_is_steady_velocity = false;
        private sbyte   _last_control_scheme = -1;
        private bool    _stabilisation_off = true, _all_engines_off = false, _control_limit_reached = false, _control_limit_message_on = false, _thrust_reduction_message_on = false;

        private  bool   _allow_extra_linear_opposition = false, _integral_cleared = false;
        private  bool[] _enable_linear_integral    = {  true,  true,  true,  true,  true,  true };
        private float[] _linear_integral           = {  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f };
        //private float[] _local_gravity_inv         = new float[6];
        //private float[] _local_linear_velocity_inv = new float[6];

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

        private void screen_info(string message, int display_time_ms, MyFontEnum font, bool controlled_only)
        {
            if (!controlled_only || MySession.Static.ControlledEntity.Entity.GetTopMostParent() == _grid)
                Sandbox.ModAPI.MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
        }

        [Conditional("DEBUG")]
        private void screen_text(string method_name, string message, int display_time_ms, bool controlled_only)
        {
            if (method_name == "")
                screen_info(string.Format("\"{0}\" {1}", _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
            else
                screen_info(string.Format("engine_control_unit.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
        }

        [Conditional("DEBUG")]
        private void screen_vector<type>(string method_name, string vector_name, type[] vector, int display_time_ms, bool controlled_only)
        {
            screen_text(method_name, string.Format("{0} = {1:F3}/{2:F3}/{3:F3}/{4:F3}/{5:F3}/{6:F3}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms, controlled_only);
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
            refresh_thruster_info_for_single_direction(_uncontrolled_thrusters);
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                refresh_thruster_info_for_single_direction(_controlled_thrusters[dir_index]);
                _calibration_needed[dir_index] = _calibration_used[dir_index];
            }
            _calibration_scheduled = true;
        }

        private void check_override_on_uncontrolled()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                _dampers_disabled[dir_index] = false;
            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                if (cur_thruster.Key.ThrustOverride > cur_thruster.Value.max_force * 0.01f)
                    _dampers_disabled[(int) cur_thruster.Value.nozzle_direction] = true;
            }
        }

        private void calculate_and_apply_torque(Vector3 desired_angular_velocity)
        {
            _torque = Vector3.Zero;
            foreach (var cur_direction in _controlled_thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                {
                    if (cur_thruster.Key.IsWorking)
                        _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
                }
            }

            foreach (var cur_thruster in _uncontrolled_thrusters)
            {
                if (cur_thruster.Key.IsWorking)
                    _torque += cur_thruster.Value.max_torque * cur_thruster.Key.CurrentStrength;
            }

            //_are_gyroscopes_saturated = _max_gyro_torque < 1.0f || torque.LengthSquared() / _max_gyro_torque_squared >= 0.75f * 0.75f;
            if (!_stabilisation_off && _manual_rotation.LengthSquared() <= 0.0001f)
            {
                float gyro_limit = _gyro_control.ResourceSink.SuppliedRatio * (_max_gyro_torque - _gyro_control.Torque.Length());
                if (gyro_limit > 1.0f)
                {
                    Vector3 gyro_torque = 5.0f * (_spherical_moment_of_inertia / _max_gyro_torque) * (desired_angular_velocity - _local_angular_velocity);
                    if (gyro_torque.LengthSquared() > 1.0f)
                        gyro_torque.Normalize();
                    gyro_torque     *= gyro_limit;
                    _torque         += gyro_torque;
                }
                /*
                if (torque.LengthSquared() <= _max_gyro_torque_squared)
                    torque  = Vector3.Zero;
                else
                    torque -= Vector3.Normalize(torque) * _max_gyro_torque;
                */
            }
            Vector3 world_torque = Vector3.Transform(_torque, _grid.WorldMatrix.GetOrientation());
            _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3.Zero, null, world_torque);
        }

        #endregion

        #region thrust calibration

        private void perform_linear_calibration()
        {
            bool  is_solution_good;
            float x = 0.0f, y = 0.0f;

            foreach (thrust_dir cur_direction in Enum.GetValues(typeof(thrust_dir)))
            {
                if (!_calibration_used[(int) cur_direction] || !_calibration_needed[(int) cur_direction])
                    continue;

                __thruster_infos.Clear();
                __thruster_infos.AddRange(_controlled_thrusters[(int) cur_direction].Values);

                for (int index = 0; index < __thruster_infos.Count; ++index)
                {
                    switch (cur_direction)
                    {
                        case thrust_dir.fore:
                        case thrust_dir.aft:
                            x = __thruster_infos[index].CoM_offset.X;
                            y = __thruster_infos[index].CoM_offset.Y;
                            break;

                        case thrust_dir.starboard:
                        case thrust_dir.port:
                            x = __thruster_infos[index].CoM_offset.Y;
                            y = __thruster_infos[index].CoM_offset.Z;
                            break;

                        case thrust_dir.dorsal:
                        case thrust_dir.ventral:
                            x = __thruster_infos[index].CoM_offset.X;
                            y = __thruster_infos[index].CoM_offset.Z;
                            break;
                    }
                    if (index >= __linear_solver.items.Count)
                        __linear_solver.items.Add(new solver_entry());
                    __linear_solver.items[index].x = x;
                    __linear_solver.items[index].y = y;
                    __linear_solver.items[index].max_value = __thruster_infos[index].max_force;
                }

                is_solution_good = __linear_solver.calculate_solution(__thruster_infos.Count);
                for (int index = 0; index < __thruster_infos.Count; ++index)
                {
                    __thruster_infos[index].thrust_limit = (is_solution_good && __linear_solver.items[index].max_value > 0.0f) 
                        ? (__linear_solver.items[index].result / __linear_solver.items[index].max_value) : 1.0f;
                    //__thruster_infos[index].enable_limit &= is_solution_good;
                    log_ECU_action("perform_linear_calibration", string.Format("{0} kN ({1})", __linear_solver.items[index].result / 1000.0f, cur_direction));
                }

                _calibration_needed[(int) cur_direction] = false;
                log_ECU_action("perform_linear_calibration", is_solution_good
                    ? string.Format("successfully calibrated {0} thrusters on {1} side", __thruster_infos.Count, cur_direction)
                    : string.Format("calibration on {0} side failed", cur_direction));
            }

            _calibration_scheduled = false;
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

        private static void recompose_vector(float[] decomposed_vector, out Vector3 result_vector)
        {
            result_vector.Z = decomposed_vector[(int) thrust_dir.fore   ] - decomposed_vector[(int) thrust_dir.aft      ];
            result_vector.X = decomposed_vector[(int) thrust_dir.port   ] - decomposed_vector[(int) thrust_dir.starboard];
            result_vector.Y = decomposed_vector[(int) thrust_dir.ventral] - decomposed_vector[(int) thrust_dir.dorsal   ];
        }

        private void apply_thrust_settings(bool reset_all_thrusters)
        {
            const float MIN_OVERRIDE = 1.001f;

            float         setting;
            int           setting_int;
            bool          enforce_min_override;
            thruster_info cur_thruster_info;

            if (reset_all_thrusters && _all_engines_off)
                return;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (reset_all_thrusters)
                    _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;

                enforce_min_override = __control_vector[dir_index] > 0.01f && _speed > 0.1f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (reset_all_thrusters || !cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
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
                    setting_int = (int) Math.Ceiling(setting);
                    if (setting_int != cur_thruster_info.prev_setting)
                    {
                        cur_thruster.Key.SetValueFloat("Override", setting);
                        cur_thruster_info.prev_setting = setting_int;
                    }
                }
            }

            _all_engines_off = reset_all_thrusters;
        }

        private sbyte initialise_linear_controls(Vector3 local_linear_velocity_vector, Vector3 local_gravity_vector)
        {
            const float DAMPING_CONSTANT = -2.0f, INTEGRAL_CONSTANT = 0.05f, DESCENDING_SPEED = 0.5f;

            _allow_extra_linear_opposition = _manual_thrust.LengthSquared() > 0.75f * 0.75f;
            decompose_vector(_manual_thrust, __control_vector);
            sbyte control_scheme = get_current_control_scheme();

            if (!_thrust_control.DampenersEnabled)
            {
                if (!_integral_cleared)
                {
                    for (int dir_index = 0; dir_index < 6; ++dir_index)
                    {
                        _enable_linear_integral[dir_index] = false;
                        _linear_integral[dir_index]        = 0.0f;
                    }
                    _integral_cleared = true;
                }
            }
            else
            {
                _integral_cleared      = false;
                Vector3 linear_damping = local_linear_velocity_vector * DAMPING_CONSTANT - (_stabilisation_off ? (local_gravity_vector * 0.8f) : local_gravity_vector);
                decompose_vector(linear_damping * _grid.Physics.Mass,            __braking_vector);
                decompose_vector(              -local_gravity_vector,         __local_gravity_inv);
                decompose_vector(      -local_linear_velocity_vector, __local_linear_velocity_inv);
                Array.Copy(__control_vector, __control_vector_copy, 6);

                int opposite_dir = 3;
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    _enable_linear_integral[dir_index] = __local_gravity_inv[dir_index] > 0.0f
                        && __control_vector_copy[dir_index] < 0.01f && __control_vector_copy[opposite_dir] < 0.01f;

                    if (_dampers_disabled[dir_index] || _controlled_thrusters[dir_index].Count == 0 || _max_force[dir_index] <= 0.0f)
                        _enable_linear_integral[dir_index] = false;
                    else if (__control_vector_copy[opposite_dir] < 0.01f)
                    {
                        __control_vector[dir_index] += (__braking_vector[dir_index] + _grid.Physics.Mass * _linear_integral[dir_index]) 
                            / _max_force[dir_index];
                        if (__control_vector[dir_index] >= 1.0f)
                        {
                            __control_vector[dir_index]        = 1.0f;
                            _enable_linear_integral[dir_index] = false;
                        }
                        if (__control_vector[dir_index] > 0.75f)
                            _allow_extra_linear_opposition = true;
                    }

                    if (!_enable_linear_integral[dir_index])
                        _linear_integral[dir_index] = 0.0f;
                    else
                    {
                        _linear_integral[dir_index] += INTEGRAL_CONSTANT * (__local_linear_velocity_inv[dir_index] - __local_linear_velocity_inv[opposite_dir]);
                        if (_stabilisation_off)
                            _linear_integral[dir_index] -= INTEGRAL_CONSTANT * DESCENDING_SPEED;
                        if (_linear_integral[dir_index] < 0.0f)
                            _linear_integral[dir_index] = 0.0f;
                    }

                    if (++opposite_dir >= 6)
                        opposite_dir = 0;
                }
            }

            return control_scheme;
        }

        private sbyte get_current_control_scheme()
        {
            const float MIN_CONTROL = 0.3f;

            sbyte result;

            if (__control_vector[(int) thrust_dir.fore] > MIN_CONTROL)
                result = 1;
            else if (__control_vector[(int) thrust_dir.aft] > MIN_CONTROL)
                result = 2;
            else
                result = 0;

            if (__control_vector[(int) thrust_dir.starboard] > MIN_CONTROL)
                result += 3;
            else if (__control_vector[(int) thrust_dir.port] > MIN_CONTROL)
                result += 6;

            if (__control_vector[(int) thrust_dir.dorsal] > MIN_CONTROL)
                result += 9;
            else if (__control_vector[(int) thrust_dir.ventral] > MIN_CONTROL)
                result += 18;

            return result;
        }

        private void adjust_thrust_for_steering(int cur_dir, int opposite_dir, Vector3 desired_angular_velocity, float thrust_limit)
        {
            const float DAMPING_CONSTANT = 5.0f;
            const float MIN_LINEAR_OPPOSITION = 0.1f, MAX_LINEAR_OPPOSITION = 0.3f;

            Vector3       angular_velocity_diff = desired_angular_velocity - _local_angular_velocity;
            float         max_linear_opposition, damping = DAMPING_CONSTANT * _grid.Physics.Mass / _max_force[cur_dir];
            thruster_info cur_thruster_info;

            __actual_force[cur_dir] = 0.0f;
            if (_max_force[cur_dir] <= 0.0f)
                return;

            if (thrust_limit < 0.0f)
                thrust_limit = 0.0f;
            max_linear_opposition = MIN_LINEAR_OPPOSITION * __control_vector[opposite_dir];
            if (_allow_extra_linear_opposition)
                max_linear_opposition += MAX_LINEAR_OPPOSITION * (1.0f - __control_vector[opposite_dir]);
            if (_max_force[opposite_dir] < _max_force[cur_dir])
                max_linear_opposition *= _max_force[opposite_dir] / _max_force[cur_dir];

            foreach (var cur_thruster in _controlled_thrusters[cur_dir])
            {
                if (!cur_thruster.Key.IsWorking)
                    continue;
                cur_thruster_info = cur_thruster.Value;
                if (cur_thruster_info.actual_max_force < 1.0f)
                    continue;

                decompose_vector(Vector3.Cross(angular_velocity_diff, cur_thruster_info.reference_vector), __linear_component);
                if (__linear_component[cur_dir] > 0.0f)
                {
                    cur_thruster_info.current_setting += damping * __linear_component[cur_dir];
                    if (__control_vector[opposite_dir] > 0.01f)
                    {
                        // Limit thrusters opposing player/ID linear input
                        if (cur_thruster_info.current_setting > max_linear_opposition)
                            cur_thruster_info.current_setting = max_linear_opposition;
                    }
                    else if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;
                }
                else if (__linear_component[opposite_dir] > 0.0f)
                {
                    cur_thruster_info.current_setting -= damping * __linear_component[opposite_dir];
                    if (cur_thruster_info.current_setting > thrust_limit)
                        cur_thruster_info.current_setting = thrust_limit;
                    if (cur_thruster_info.current_setting < 0.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else if (__control_vector[opposite_dir] > 0.01f && cur_thruster_info.current_setting > max_linear_opposition)
                    {
                        // Limit thrusters opposing player/ID linear input
                        cur_thruster_info.current_setting = max_linear_opposition;
                    }
                }

                __actual_force[cur_dir] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
            }
        }

        // Ensures that resulting linear force doesn't exceed player/ID input (to prevent undesired drift when turning)
        void normalise_thrust()
        {
            int   opposite_dir = 3;
            float new_force_ratio, linear_force = 0.0f, requested_force = 0.0f, force;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (__actual_force[dir_index] >= 1.0f && __actual_force[dir_index] - __requested_force[dir_index] > __actual_force[opposite_dir])
                {
                    new_force_ratio = (__actual_force[opposite_dir] + __requested_force[dir_index]) / __actual_force[dir_index];
                    foreach (var cur_thruster in _controlled_thrusters[dir_index])
                        cur_thruster.Value.current_setting *= new_force_ratio;
                    __actual_force[dir_index] *= new_force_ratio;
                }
                if (__actual_force[opposite_dir] >= 1.0f && __actual_force[opposite_dir] - __requested_force[opposite_dir] > __actual_force[dir_index])
                {
                    new_force_ratio = (__actual_force[dir_index] + __requested_force[opposite_dir]) / __actual_force[opposite_dir];
                    foreach (var cur_thruster in _controlled_thrusters[opposite_dir])
                        cur_thruster.Value.current_setting *= new_force_ratio;
                    __actual_force[opposite_dir] *= new_force_ratio;
                }

                force            =    __actual_force[dir_index] -    __actual_force[opposite_dir];
                linear_force    += force * force;
                force            = __requested_force[dir_index] + __requested_force[opposite_dir];
                requested_force += force * force;
                ++opposite_dir;
            }

            if ((!_thrust_control.DampenersEnabled || _speed < 1.0f) && _manual_thrust.LengthSquared() < 0.0001f)
                _thrust_reduction = 0;
            else
            {
                linear_force      = (float) Math.Sqrt(   linear_force);
                requested_force   = (float) Math.Sqrt(requested_force);
                _thrust_reduction = (  int) ((1.0f - linear_force / requested_force) * 100.0f + 0.5f);
            }
        }

        private bool adjust_trim_setting(sbyte control_scheme, out Vector3 desired_angular_velocity, out float thrust_limit)
        {
            const float ANGULAR_INTEGRAL_COEFF = -0.05f, ANGULAR_DERIVATIVE_COEFF = -0.01f, MAX_TRIM = 1.0f, THRUST_CUTOFF_TRIM = 0.9f;

            bool    update_inverse_world_matrix = false;
            float   trim_change;
            Vector3 local_angular_acceleration  = (_local_angular_velocity - _prev_angular_velocity) / MyEngineConstants.UPDATE_STEP_SIZE_IN_SECONDS;
            _prev_angular_velocity = _local_angular_velocity;

            decompose_vector(                      _manual_rotation,       __steering_input);
            decompose_vector(               _local_angular_velocity,     __angular_velocity);
            decompose_vector(            local_angular_acceleration, __angular_acceleration);
            decompose_vector(_torque / _spherical_moment_of_inertia, __nominal_acceleration);
            thrust_limit     = 1.0f;
            int opposite_dir = 3;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (__steering_input[opposite_dir] > 0.01f)
                    __steering_output[dir_index] = 0.0f;
                else if (__steering_input[dir_index] > 0.01f)
                {
                    __steering_output[dir_index] = (__angular_velocity[dir_index] < 0.1f) ? 
                          (__steering_input[dir_index] * 15.0f) 
                        : (__angular_velocity[dir_index] + __steering_input[dir_index] * 2.0f);
                    __steering_output[dir_index] += _last_trim[dir_index];
                    _enable_integral[dir_index]   = _enable_integral[opposite_dir] = false;
                    _current_trim[dir_index]      =    _current_trim[opposite_dir] = 0.0f;
                    update_inverse_world_matrix   = true;
                }
                else
                {
                    if (_stabilisation_off)
                        _current_trim[dir_index] = _last_trim[dir_index] = 0.0f;
                    else if (_enable_integral[dir_index])
                    {
                        if (_restrict_integral[dir_index] && __angular_velocity[dir_index] < 0.001f)
                        {
                            _restrict_integral[dir_index] = false;
                            update_inverse_world_matrix   = true;
                        }

                        if (!_restrict_integral[dir_index] || __angular_acceleration[opposite_dir] < 0.005f)
                        {
                            trim_change = ANGULAR_INTEGRAL_COEFF * ((!_restrict_integral[dir_index] || __angular_velocity[dir_index] < 0.01f) ? __angular_velocity[dir_index] : 0.01f);
                            // _nominal_acceleration is used to combat trim "stickiness" caused by rotational friction
                            if (__angular_velocity[dir_index] < 0.0001f && __angular_velocity[opposite_dir] < 0.0001f)
                                 trim_change += ANGULAR_DERIVATIVE_COEFF * __nominal_acceleration[dir_index];

                            if (_current_trim[dir_index] > 0.0f)
                            {
                                // trim_change is always negative or zero
                                _current_trim[dir_index] += trim_change;
                                if (_current_trim[dir_index] < 0.0f)
                                {
                                    _current_trim[opposite_dir] = -_current_trim[dir_index];
                                    _current_trim[   dir_index] = 0.0f;
                                }
                            }
                            else
                            {
                                // trim_change is always negative or zero
                                if (_current_trim[opposite_dir] < MAX_TRIM)
                                    _current_trim[opposite_dir] -= trim_change;
                                else
                                    _control_limit_reached = true;
                            }
                        }

                        if (_current_trim[opposite_dir] <= 0.0f)
                        {
                            _last_trim[dir_index] = (_last_trim[dir_index] <= _current_trim[dir_index]) ? 
                                    _current_trim[dir_index]
                                : ((_current_trim[dir_index] + _last_trim[dir_index]) / 2.0f);
                        }
                        else
                            _last_trim[dir_index] = 0.0f;
                    }
                    else
                    {
                        _enable_integral[  dir_index] = update_inverse_world_matrix = true;
                        _restrict_integral[dir_index] =  !_stabilisation_off && __angular_velocity[dir_index] > 0.0f;
                        _current_trim[     dir_index] = (!_stabilisation_off && control_scheme == _last_control_scheme) ? _last_trim[dir_index] : 0.0f;
                    }

                    __steering_output[dir_index] = __angular_velocity[opposite_dir] + _current_trim[dir_index];
                }

                if (thrust_limit > 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * _current_trim[dir_index])
                    thrust_limit = 1.0f - (1.0f / THRUST_CUTOFF_TRIM) * _current_trim[dir_index];
                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            recompose_vector(__steering_output, out desired_angular_velocity);
            desired_angular_velocity += ANGULAR_DERIVATIVE_COEFF * local_angular_acceleration;
            _last_control_scheme      = control_scheme;
            if (thrust_limit < 0.0f)
                thrust_limit = 0.0f;
            return update_inverse_world_matrix;
        }

        private void handle_thrust_control(out Vector3 desired_angular_velocity)
        {
            // Using "fixed" (it changes orientation only when the player steers a ship) inverse rotation matrix here to 
            // prevent Dutch Roll-like tendencies at high speeds
            Vector3 local_linear_velocity = Vector3.Transform(_grid.Physics.LinearVelocity, _inverse_world_rotation_fixed);

            Matrix inverse_world_rotation = _inverse_world_transform.GetOrientation();
            Vector3 local_gravity         = Vector3.Transform(_grid.Physics.Gravity, inverse_world_rotation);
            _local_angular_velocity       = Vector3.Transform(_grid.Physics.AngularVelocity, inverse_world_rotation);
            _speed = local_linear_velocity.Length();

            float thrust_limit;
            _stabilisation_off = true;
            foreach (var cur_direction in _controlled_thrusters)
                _stabilisation_off &= cur_direction.Count == 0;
            bool RC_stabilisation = __current_controller is MyRemoteControl && __current_controller.CubeGrid == _grid && __current_controller.HorizonIndicatorEnabled;
            _stabilisation_off = (!_grid.HasMainCockpit() && !RC_stabilisation) || _stabilisation_off && _max_gyro_torque < 0.0001f * _spherical_moment_of_inertia;
            sbyte control_scheme              = initialise_linear_controls(local_linear_velocity, local_gravity);
            bool  update_inverse_world_matrix = adjust_trim_setting(control_scheme, out desired_angular_velocity, out thrust_limit);
            // Update fixed inverse rotation matrix when angle exceeds 11 degrees or speed is low 
            // (decoupling inertia dampers' axes from ship orientation isn't needed at low velocities)
            if (update_inverse_world_matrix || _speed <= 20.0f || Vector3.Dot(_inverse_world_rotation_fixed.Forward, inverse_world_rotation.Forward) < 0.98f)
                _inverse_world_rotation_fixed = inverse_world_rotation;

            _new_mode_is_steady_velocity    = true; 
            _allow_extra_linear_opposition |= _manual_rotation.LengthSquared() > 0.0001f || _local_angular_velocity.LengthSquared() > 0.0003f;
            int opposite_dir                = 3;
            thruster_info cur_thruster_info;
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                if (__control_vector[dir_index] > 0.05f)
                    _new_mode_is_steady_velocity = false;
                __requested_force[dir_index] = __actual_force[dir_index] = 0.0f;
                foreach (var cur_thruster in _controlled_thrusters[dir_index])
                {
                    cur_thruster_info  = cur_thruster.Value;
                    if (!cur_thruster.Key.IsWorking || cur_thruster_info.actual_max_force < 1.0f)
                        cur_thruster_info.current_setting = 0.0f;
                    else
                    {
                        cur_thruster_info.current_setting = __control_vector[dir_index];
                        __requested_force[dir_index]     += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                        if (!_stabilisation_off && cur_thruster_info.enable_limit)
                            cur_thruster_info.current_setting *= cur_thruster_info.thrust_limit;
                        __actual_force[dir_index] += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                    }
                }
                if (!_gyro_override_active && !_dampers_disabled[dir_index])
                    adjust_thrust_for_steering(dir_index, opposite_dir, desired_angular_velocity, thrust_limit);
                if (++opposite_dir >= 6)
                    opposite_dir = 0;
            }

            normalise_thrust();
            apply_thrust_settings(reset_all_thrusters: false);
        }

        private static void set_thruster_reference_vector(thruster_info thruster, Vector3 reference)
        {
            thruster.reference_vector = reference;
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
            bool     changes_made = false, enable_limit;
            int      dir_index;

            for (dir_index = 0; dir_index < 6; ++dir_index)
            {
                MyThrust cur_thruster;
                Dictionary<MyThrust, thruster_info> cur_direction = _controlled_thrusters[dir_index];

                _calibration_needed[dir_index] = _calibration_used[dir_index] = false;
                __thrusters_copy.Clear();
                __thrusters_copy.AddRange(cur_direction.Keys);
                __thruster_infos.Clear();
                __thruster_infos.AddRange(cur_direction.Values);
                for (int index = 0; index < __thrusters_copy.Count; ++index)
                {
                    cur_thruster = __thrusters_copy[index];
                    cur_thruster.CustomName.ToUpperTo(__thruster_name);
                    if (!cur_thruster.IsWorking || cur_direction[cur_thruster].actual_max_force < 0.01f * cur_direction[cur_thruster].max_force || !__thruster_name.ContainsRCSTag())
                    {
                        cur_thruster.SetValueFloat("Override", 0.0f);
                        _max_force[dir_index] -= __thruster_infos[index].max_force;
                        _uncontrolled_thrusters.Add(cur_thruster, __thruster_infos[index]);
                        cur_thruster.ThrustOverrideChanged += on_thrust_override_changed;
                        cur_direction.Remove(cur_thruster);
                        changes_made = _calibration_needed[dir_index] = true;
                    }
                    else
                    {
                        enable_limit = __thruster_name.ContainsSTATTag();
                        _calibration_used[dir_index] |= enable_limit;
                        if (__thruster_infos[index].enable_limit != enable_limit)
                        {
                            __thruster_infos[index].enable_limit = enable_limit;
                            _calibration_scheduled              |= enable_limit;
                            _calibration_needed[dir_index]      |= enable_limit;
                        }
                    }
                }
                _calibration_needed[dir_index] &= _calibration_used[dir_index];
            }

            __thrusters_copy.Clear();
            __thrusters_copy.AddRange(_uncontrolled_thrusters.Keys);
            foreach (var cur_thruster in __thrusters_copy)
            {
                cur_thruster.CustomName.ToUpperTo(__thruster_name);
                if (cur_thruster.IsWorking  && _uncontrolled_thrusters[cur_thruster].actual_max_force > 0.01f * _uncontrolled_thrusters[cur_thruster].max_force && __thruster_name.ContainsRCSTag())
                {
                    dir_index = (int) _uncontrolled_thrusters[cur_thruster].nozzle_direction;
                    _controlled_thrusters[dir_index].Add(cur_thruster, _uncontrolled_thrusters[cur_thruster]);
                    cur_thruster.ThrustOverrideChanged -= on_thrust_override_changed;
                    _uncontrolled_thrusters.Remove(cur_thruster);
                    _max_force[dir_index]         += _controlled_thrusters[dir_index][cur_thruster].max_force;
                    changes_made                   = true;
                    _calibration_needed[dir_index] = _calibration_used[dir_index];
                }
            }

            if (changes_made)
            {
                if (_current_mode_is_steady_velocity)
                    update_reference_vectors_for_steady_velocity_mode();
                else
                    update_reference_vectors_for_accelerating_mode();
                check_override_on_uncontrolled();
                _calibration_scheduled = true;
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
            }

            if (_calibration_scheduled)
                perform_linear_calibration();
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
            new_thruster.max_torque       = Vector3.Cross(new_thruster.CoM_offset, thruster.ThrustForwardVector * (-new_thruster.max_force));
            new_thruster.nozzle_direction = get_nozzle_orientation(thruster);
            new_thruster.thrust_limit     = 1.0f;
            new_thruster.enable_limit     = false;
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
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    if (_controlled_thrusters[dir_index].ContainsKey(thruster))
                    {
                        thruster_found = _calibration_scheduled = true;
                        _calibration_needed[dir_index]          = _calibration_used[dir_index];
                        _controlled_thrusters[dir_index].Remove(thruster);
                        log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", thruster.CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
                        break;
                    }
                }
            }
            Debug.Assert(thruster_found, "TT&DT engine_control_unit.dispose_thruster ERROR: " + thruster.CustomName + " [" + thruster.EntityId + "] hasn't been registered");
        }

        private void calc_spherical_moment_of_inertia()
        {
            Vector3 grid_dim             = (_grid.Max - _grid.Min + Vector3I.One) * _grid.GridSize;
            float grid_volume            = Math.Abs(grid_dim.X * grid_dim.Y * grid_dim.Z);
            float reference_radius       = (float) Math.Pow(grid_volume / (4.0 / 3.0 * Math.PI), 1.0 / 3.0);
            _spherical_moment_of_inertia = 0.4f * _grid.Physics.Mass * reference_radius * reference_radius;
            log_ECU_action("calc_spherical_moment_of_inertia", string.Format("volume = {0} m3, radius = {1} m, SMoI = {2} t*m2", grid_volume, reference_radius, _spherical_moment_of_inertia / 1000.0f));
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
            _grid = grid;
            _grid.OnBlockAdded   += on_block_added;
            _grid.OnBlockRemoved += on_block_removed;
            _inverse_world_transform      = _grid.PositionComp.WorldMatrixNormalizedInv;
            _grid_CoM_location            = Vector3D.Transform(_grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            _inverse_world_rotation_fixed = _inverse_world_transform.GetOrientation();
            calc_spherical_moment_of_inertia();

            _gyro_control         = _grid.GridSystems.GyroSystem;
            Type gyro_system_type = _gyro_control.GetType();
            _max_gyro_torque_ref  = gyro_system_type.GetField("m_maxGyroForce"    , BindingFlags.Instance | BindingFlags.NonPublic);
            _gyro_override_ref    = gyro_system_type.GetField("m_maxOverrideForce", BindingFlags.Instance | BindingFlags.NonPublic);

            IMyGridTerminalSystem grid_control = _grid.GridSystems.TerminalSystem;
            var thruster_list = new List<IMyTerminalBlock>();
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
                if (__current_controller != null && __current_controller.CubeGrid == _grid)
                {
                    Matrix cockpit_matrix;
                    __current_controller.Orientation.GetMatrix(out cockpit_matrix);
                    _manual_thrust     = Vector3.Clamp(Vector3.Transform(__current_controller.MoveIndicator, cockpit_matrix), -Vector3.One, Vector3.One);
                    _manual_rotation.X = __current_controller.RotationIndicator.X * (-0.05f);
                    _manual_rotation.Y = __current_controller.RotationIndicator.Y * (-0.05f);
                    _manual_rotation.Z = __current_controller.RollIndicator       * (-0.2f );
                    _manual_rotation   = Vector3.Transform(_manual_rotation, cockpit_matrix);
                }

                if (   _manual_rotation.LengthSquared() < 0.0001f && _manual_thrust.LengthSquared() < 0.0001f
                    && _grid.Physics.AngularVelocity.LengthSquared() < 1.0E-6f && _grid.Physics.Gravity.LengthSquared() < 0.01f
                    && (!_thrust_control.DampenersEnabled || _grid.Physics.LinearVelocity.LengthSquared() < 0.01f))
                {
                    _thrust_reduction = 0;
                    apply_thrust_settings(reset_all_thrusters: true);
                }
                else
                {
                    Vector3 desired_angular_velocity;
                    handle_thrust_control(out  desired_angular_velocity);
                    calculate_and_apply_torque(desired_angular_velocity);
                }
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
                calc_spherical_moment_of_inertia();
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

            if (__current_controller == null || __current_controller.CubeGrid != _grid)
            {
                if (_control_limit_message_on)
                {
                    MyHud.Notifications.Remove(__control_limit_message);
                    _control_limit_message_on = false;
                }
                if (_thrust_reduction_message_on)
                {
                    MyHud.Notifications.Remove(_thrust_reduction_message);
                    _thrust_reduction_message_on = false;
                }
            }
            else
            {
                if (_control_limit_reached != _control_limit_message_on)
                {
                    _control_limit_message_on = _control_limit_reached;
                    if (!_control_limit_reached)
                        MyHud.Notifications.Remove(__control_limit_message);
                    else
                        MyHud.Notifications.Add(__control_limit_message);
                }
                if ((_thrust_reduction >= 5) != _thrust_reduction_message_on)
                {
                    _thrust_reduction_message_on = _thrust_reduction >= 5;
                    if (_thrust_reduction_message_on)
                        MyHud.Notifications.Add(_thrust_reduction_message);
                    else
                        MyHud.Notifications.Remove(_thrust_reduction_message);
                }
                if (_thrust_reduction_message_on)
                {
                    _thrust_reduction_message.Font = (_thrust_reduction > 30) ? MyFontEnum.Red : MyFontEnum.White;
                    _thrust_reduction_message.SetTextFormatArguments(_thrust_reduction);
                }
            }
            _control_limit_reached = false;

            if (!(__current_controller is MyCockpit) && !(__current_controller is MyRemoteControl) || __current_controller.CubeGrid != _grid)
                _status_displayed = _RC_status_displayed = false;
            else
            {
                if (__current_controller is MyRemoteControl)
                {
                    if (!_RC_status_displayed)
                    {
                        if (__current_controller.HorizonIndicatorEnabled)
                            screen_info("Active RC stabilisation is enabled. Uncheck \"Show horizon and altitude\" to disable", 5000, MyFontEnum.White, controlled_only: false);
                        else
                            screen_info("Active RC stabilisation is disabled. Tick \"Show horizon and altitude\" to enable", 5000, MyFontEnum.White, controlled_only: false);
                        _RC_status_displayed = true;
                    }
                }
                else if (!_status_displayed)
                {
                    if (_grid.HasMainCockpit())
                        screen_info("Active stabilisation is enabled. Uncheck \"Main Cockpit\" to disable", 5000, MyFontEnum.White, controlled_only: true);
                    else
                        screen_info("Active stabilisation is disabled. Set up a main cockpit to enable", 5000, MyFontEnum.White, controlled_only: true);
                    _status_displayed = true;
                }
            }
        }

        public static void refresh_ship_controller_4Hz()
        {
            __current_controller = MySession.Static.ControlledEntity as MyShipController;
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
