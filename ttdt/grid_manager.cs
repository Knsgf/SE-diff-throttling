using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Sandbox.Game.Entities;
using Sandbox.Game.Entities.Cube;
using Sandbox.Game.GameSystems;
using VRage.Game.Entity;
using VRage.Utils;

namespace thruster_torque_and_differential_throttling
{
    static class grid_manager
    {
        private static Dictionary<MyCubeGrid, engine_control_unit> grids_with_ECU = new Dictionary<MyCubeGrid, engine_control_unit>();
        private static HashSet<MyCubeGrid> uncontrolled_grids = new HashSet<MyCubeGrid>();
        private static List<MyCubeGrid> 
            grids_with_ECU_copy     = new List<MyCubeGrid>(), 
            uncontrolled_grids_copy = new List<MyCubeGrid>();

        #region Private methods

        [Conditional("DEBUG")]
        private static void log_grid_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT \tgrid_manager.{0}(): {1}", method_name, message));
            MyLog.Default.WriteLine(string.Format("TT&DT \ttotal grids: {0} ({1} with ECU)", uncontrolled_grids.Count + grids_with_ECU.Count, grids_with_ECU.Count));
        }

        private static void check_grid_control_changed(MyCubeGrid grid)
        {
            if (grid.IsStatic || grid.Physics == null || grid.Components.Get<MyEntityThrustComponent>() == null)
            {
                if (grids_with_ECU.ContainsKey(grid))
                {
                    grids_with_ECU[grid].Dispose();
                    grids_with_ECU.Remove(grid);
                    uncontrolled_grids.Add(grid);
                    log_grid_action("check_grid_control_changed", string.Format("detached ECU from grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (uncontrolled_grids.Contains(grid))
            {
                uncontrolled_grids.Remove(grid);
                grids_with_ECU.Add(grid, new engine_control_unit(grid));
                log_grid_action("check_grid_control_changed", string.Format("attached ECU to grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }

        #endregion

        public static void add_grid(MyCubeGrid grid)
        {
            if (grid.IsStatic || grid.Physics == null)
            {
                if (!uncontrolled_grids.Contains(grid))
                {
                    uncontrolled_grids.Add(grid);
                    log_grid_action("add_grid", string.Format("added grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (!grids_with_ECU.ContainsKey(grid))
            {
                grids_with_ECU.Add(grid, new engine_control_unit(grid));
                log_grid_action("add_grid", string.Format("added and attached ECU to grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }

        public static void remove_grid(MyCubeGrid grid)
        {
            if (grid.IsStatic || grid.Physics == null || grid.Components.Get<MyEntityThrustComponent>() == null)
            {
                if (uncontrolled_grids.Contains(grid))
                {
                    uncontrolled_grids.Remove(grid);
                    log_grid_action("remove_grid", string.Format("removed grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (grids_with_ECU.ContainsKey(grid))
            {
                grids_with_ECU[grid].Dispose();
                grids_with_ECU.Remove(grid);
                log_grid_action("remove_grid", string.Format("removed and detached ECU from grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }

        public static void handle_60Hz()
        {
            foreach (var cur_ECU in grids_with_ECU.Values)
                cur_ECU.handle_60Hz();
        }

        public static void handle_4Hz()
        {
            foreach (var cur_ECU in grids_with_ECU.Values)
                cur_ECU.handle_4Hz();
        }

        public static void handle_2s_period()
        {
            grids_with_ECU_copy.Clear();
            grids_with_ECU_copy.AddRange(grids_with_ECU.Keys);
            uncontrolled_grids_copy.Clear();
            uncontrolled_grids_copy.AddRange(uncontrolled_grids);

            foreach (var cur_grid in grids_with_ECU_copy)
                check_grid_control_changed(cur_grid);
            foreach (var cur_grid in uncontrolled_grids_copy)
                check_grid_control_changed(cur_grid);

            foreach (var cur_ECU in grids_with_ECU.Values)
                cur_ECU.handle_2s_period();
        }
    }
}
