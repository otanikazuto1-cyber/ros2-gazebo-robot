#!/usr/bin/env python3
from pathlib import Path
world = '''<?xml version="1.0" ?>
<sdf version="1.9">
<world name="arena">
  <gravity>0 0 -9.8</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <scene><ambient>0.8 0.8 0.8 1</ambient><background>0.85 0.9 1 1</background></scene>
  <light type="directional" name="sun"><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.5 -0.5 -1</direction></light>
  <model name="floor"><static>true</static><link name="base"><collision name="c"><geometry><box><size>7 7 0.1</size></box></geometry></collision><visual name="v"><geometry><box><size>7 7 0.1</size></box></geometry><material><ambient>0.75 0.78 0.72 1</ambient><diffuse>0.75 0.78 0.72 1</diffuse></material></visual></link><pose>0 0 -0.05 0 0 0</pose></model>
'''
for name, x, y, sx, sy in [('north',0,3,6.2,.2),('south',0,-3,6.2,.2),('east',3,0,.2,6.2),('west',-3,0,.2,6.2)]:
    world += f'''  <model name="fence_{name}"><static>true</static><pose>{x} {y} 0.5 0 0 0</pose><link name="wall"><collision name="c"><geometry><box><size>{sx} {sy} 1</size></box></geometry></collision><visual name="v"><geometry><box><size>{sx} {sy} 1</size></box></geometry><material><ambient>0.1 0.35 0.8 1</ambient><diffuse>0.1 0.35 0.8 1</diffuse></material></visual></link></model>\n'''
world += '</world></sdf>\n'
p = Path(__file__).with_name('fence_arena.sdf')
p.write_text(world, encoding='utf-8')
print('Generated', p)
