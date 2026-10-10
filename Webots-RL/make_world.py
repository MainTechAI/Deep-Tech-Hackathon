"""Generate the self-contained Webots world (no downloaded PROTO dependencies)."""
from pathlib import Path


def box(name, position, size, color="0.5 0.5 0.5"):
    return f'''DEF {name} Solid {{
  translation {position}
  children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }} geometry Box {{ size {size} }} }} ]
  boundingObject Box {{ size {size} }}
}}
'''


def wheel(name, y):
    return f'''HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {y} -0.0325 }}
      device [ RotationalMotor {{ name "{name} wheel" maxVelocity 2 maxTorque 0.15 }} ]
      endPoint Solid {{
        translation 0 {y} -0.0325
        rotation 1 0 0 1.57079632679
        children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 1 metalness 0 }} geometry Cylinder {{ radius 0.0325 height 0.025 }} }} ]
        name "{name} wheel solid"
        boundingObject Cylinder {{ radius 0.0325 height 0.025 }}
        physics Physics {{ density -1 mass 0.04 }}
      }}
    }}'''


def world():
    text = '''#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 32 randomSeed 1 }
Viewpoint { orientation 0.3574 0.8629 0.3574 1.7178 position 4 -4 5 }
Background { skyColor [ 0.12 0.15 0.2 ] }
DirectionalLight { direction -1 -1 -2 intensity 1 }
'''
    text += box("FLOOR", "0 0 -0.05", "4.2 4.2 0.1", "0.7 0.7 0.7")
    for i, (position, size) in enumerate([
        ("-2.05 0 0.15", "0.1 4.2 0.3"), ("2.05 0 0.15", "0.1 4.2 0.3"),
        ("0 -2.05 0.15", "4 0.1 0.3"), ("0 2.05 0.15", "4 0.1 0.3")]):
        text += box(f"WALL_{i}", position, size)
    for i in range(6):
        text += box(f"BLOCK_{i}", f"{(i % 3 - 1) * 0.8} {(i // 3 - 0.5) * 1.2} 0.15", "0.3 0.3 0.3", "0.7 0.3 0.1")
    text += '''Pose {
  translation 1.5 1.5 0.002
  children [ Shape { appearance PBRAppearance { baseColor 0.1 0.8 0.3 metalness 0 } geometry Cylinder { radius 0.18 height 0.004 } } ]
}
DEF ROVER Robot {
  translation -1.5 -1.5 0.065
  rotation 0 0 1 0.785398
  supervisor TRUE
  controller "navigation"
  name "navigation rover"
  children [
    Shape { appearance PBRAppearance { baseColor 0.1 0.4 0.8 metalness 0 } geometry Box { size 0.2 0.12 0.045 } }
'''
    text += wheel("left", 0.07) + "\n" + wheel("right", -0.07)
    text += '''
    Solid {
      translation -0.075 0 -0.05
      children [ Shape { geometry Sphere { radius 0.015 } } ]
      contactMaterial "caster"
      boundingObject Sphere { radius 0.015 }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 0 1 anchor 0.075 0 0.05 }
      device [ RotationalMotor { name "sonar pan" maxVelocity 6 maxTorque 0.1 } ]
      endPoint Solid {
        translation 0.075 0 0.05
        children [
          DistanceSensor {
            name "sonar"
            type "sonar"
            numberOfRays 5
            aperture 0.261799
            lookupTable [ 0 0 0, 2.5 2.5 0 ]
          }
          Shape { geometry Box { size 0.025 0.04 0.02 } }
        ]
        boundingObject Box { size 0.025 0.04 0.02 }
        physics Physics { density -1 mass 0.01 }
      }
    }
  ]
  boundingObject Box { size 0.2 0.12 0.045 }
  physics Physics { density -1 mass 0.4 }
}
'''
    # Caster slides; drive wheel/floor contacts keep Webots default friction.
    return text.replace("randomSeed 1", 'randomSeed 1 contactProperties [ ContactProperties { material1 "caster" coulombFriction [ 0.01 ] } ]')


if __name__ == "__main__":
    destination = Path(__file__).parent / "worlds" / "navigation.wbt"
    destination.parent.mkdir(exist_ok=True)
    destination.write_text(world(), encoding="utf-8")
    print(destination)
