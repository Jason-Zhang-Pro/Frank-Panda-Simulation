"""Draw TCP path in the existing Gazebo GUI, using its marker service."""
import subprocess


def draw_path(positions):
    # GUI MarkerManager owns /marker; absence in headless mode is harmless.
    points = positions[::max(1, len(positions)//250)]
    if points[-1] != positions[-1]:
        points.append(positions[-1])
    request = ('ns: "panda_planner", id: 1, action: ADD_MODIFY, type: LINE_STRIP, '
               'scale {x: 1 y: 1 z: 1} pose {orientation {w: 1}} '
               'material {ambient {r: 0.0 g: 1.0 b: 0.4 a: 1.0} '
               'diffuse {r: 0.0 g: 1.0 b: 0.4 a: 1.0}} ')
    request += ' '.join('point {x: %.8f y: %.8f z: %.8f}' % tuple(q) for q in points)
    request = 'marker {' + request + '}'
    result = subprocess.run([
        'ign', 'service', '-s', '/marker_array', '--reqtype', 'ignition.msgs.Marker_V',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', request,
    ], capture_output=True, text=True, timeout=3)
    return result.returncode == 0 and 'data: true' in result.stdout
