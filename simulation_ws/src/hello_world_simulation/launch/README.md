Use empty.world.erb to generate world file with parameterized physics profiles

```
# 1. generate world file
erb real_time_update_rate=1000,2000,5000,10000 real_time_factor=1,2,5,10,20 max_step_size=0.001,0.0001 ode_solver_iters=20,50,100 worlds/empty.world.erb > worlds/empty.world

# 2. build
colcon build

# 3. set profile using envvar
export PHYSICS_PROFILE='rtu=10000,rtf=5,mss=0.001,iters=100'

# 4. launch
roslaunch hello_world_simulation empty_world.launch
```
