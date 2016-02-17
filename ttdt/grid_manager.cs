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
        private static HashSet<MyCubeGrid> physicsless_grids = new HashSet<MyCubeGrid>();

        #region Private methods

        [Conditional("DEBUG")]
        private static void log_grid_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT \tgrid_manager.{0}(): {1}", method_name, message));
            MyLog.Default.WriteLine(string.Format("TT&DT \ttotal grids: {0} ({1} with ECU)", physicsless_grids.Count + grids_with_ECU.Count, grids_with_ECU.Count));
        }

        private static void on_grid_physics_changed(MyEntity entity)
        {
            var grid = (MyCubeGrid) entity;
            if (grid.IsStatic || grid.Physics == null || grid.Components.Get<MyEntityThrustComponent>() == null)
            {
                if (grids_with_ECU.ContainsKey(grid))
                {
                    grids_with_ECU[grid].Dispose();
                    grids_with_ECU.Remove(grid);
                    physicsless_grids.Add(grid);
                    log_grid_action("on_grid_physics_changed", string.Format("detached ECU from grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (physicsless_grids.Contains(grid))
            {
                physicsless_grids.Remove(grid);
                grids_with_ECU.Add(grid, new engine_control_unit(grid));
                log_grid_action("on_grid_physics_changed", string.Format("attached ECU to grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }

        private static void on_grid_changed(MySlimBlock block)
        {
            if (block.FatBlock != null && block.FatBlock is MyThrust)
                on_grid_physics_changed(block.CubeGrid);
        }

        #endregion

        public static void add_grid(MyCubeGrid grid)
        {
            if (grid.IsStatic || grid.Physics == null)
            {
                if (!physicsless_grids.Contains(grid))
                {
                    physicsless_grids.Add(grid);
                    grid.OnPhysicsChanged += on_grid_physics_changed;
                    grid.OnBlockAdded     += on_grid_changed;
                    grid.OnBlockRemoved   += on_grid_changed;
                    log_grid_action("add_grid", string.Format("added grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (!grids_with_ECU.ContainsKey(grid))
            {
                grids_with_ECU.Add(grid, new engine_control_unit(grid));
                grid.OnPhysicsChanged += on_grid_physics_changed;
                grid.OnBlockAdded     += on_grid_changed;
                grid.OnBlockRemoved   += on_grid_changed;
                log_grid_action("add_grid", string.Format("added and attached ECU to grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }

        public static void remove_grid(MyCubeGrid grid)
        {
            if (grid.IsStatic || grid.Physics == null || grid.Components.Get<MyEntityThrustComponent>() == null)
            {
                if (physicsless_grids.Contains(grid))
                {
                    physicsless_grids.Remove(grid);
                    grid.OnPhysicsChanged -= on_grid_physics_changed;
                    grid.OnBlockAdded     -= on_grid_changed;
                    grid.OnBlockRemoved   -= on_grid_changed;
                    log_grid_action("remove_grid", string.Format("removed grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
                }
            }
            else if (grids_with_ECU.ContainsKey(grid))
            {
                grids_with_ECU[grid].Dispose();
                grids_with_ECU.Remove(grid);
                grid.OnPhysicsChanged -= on_grid_physics_changed;
                grid.OnBlockAdded     -= on_grid_changed;
                grid.OnBlockRemoved   -= on_grid_changed;
                log_grid_action("remove_grid", string.Format("removed and detached ECU from grid \"{0}\" [{1}]", grid.DisplayName, grid.EntityId));
            }
        }
    }
}
