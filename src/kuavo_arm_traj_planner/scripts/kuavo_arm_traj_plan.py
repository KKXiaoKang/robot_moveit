#!/usr/bin/env python3

from interface import Interface
from scene import Scene


if __name__ == "__main__":    
    scene = Scene()
    scene.init_scene()
    
    intreface = Interface()
    intreface.sub_camera()
