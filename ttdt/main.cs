using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Sandbox.Game.Entities;
using VRage.Game.Entity;
using VRage.Plugins;
using VRage.Utils;

namespace thruster_torque_and_differential_throttling
{
    public class main: IPlugin
    {
        #region Private methods
        private static void log_event(string event_name, MyEntity entity)
        {
            MyLog.Default.WriteLine(string.Format("TT&DT main.{0}(): {1} {2} \"{3}\" [{4}]", event_name, entity, entity.Name, entity.DisplayName, entity.EntityId));
        }

        private void on_entity_added(MyEntity entity)
        {
            log_event("on_entity_added", entity);
            var cube_grid = entity as MyCubeGrid;
            if (cube_grid != null)
                grid_manager.add_grid(cube_grid);
        }

        private void on_entity_deleted(MyEntity entity)
        {
            log_event("on_entity_deleted", entity);
            var cube_grid = entity as MyCubeGrid;
            if (cube_grid != null)
                grid_manager.remove_grid(cube_grid);
        }

        private void on_entity_removed(MyEntity entity)
        {
            log_event("on_entity_removed", entity);
            var cube_grid = entity as MyCubeGrid;
            if (cube_grid != null)
                grid_manager.remove_grid(cube_grid);
        }
        #endregion

        public void Dispose()
        {
            MyLog.Default.WriteLine("TT&DT main.Dispose()");

            MyEntities.OnEntityAdd    -= on_entity_added;
            MyEntities.OnEntityDelete -= on_entity_deleted;
            MyEntities.OnEntityRemove -= on_entity_removed;
        }

        public void Init(object gameInstance)
        {
            MyLog.Default.WriteLine("TT&DT main.Init()");

            MyEntities.OnEntityAdd    += on_entity_added;
            MyEntities.OnEntityDelete += on_entity_deleted;
            MyEntities.OnEntityRemove += on_entity_removed;
        }

        public void Update()
        {
        }
    }
}
