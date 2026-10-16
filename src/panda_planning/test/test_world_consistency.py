"""Gazebo visuals/physics and planner must use matching scene geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def test_world_consistency():
    folder=Path(get_package_share_directory('panda_planning'))/'worlds'
    urdf=ET.parse(folder/'world.urdf').getroot()
    world=ET.parse(folder/'world.sdf').find('world')
    names={link.get('name') for link in urdf.findall('link')}-{'obstacle_world_root'}
    assert names=={model.get('name') for model in world.findall('model')}
    for name in names:
        link=urdf.find(f"link[@name='{name}']")
        model=world.find(f"model[@name='{name}']")
        joint=next(j for j in urdf.findall('joint') if j.find('child').get('link')==name)
        origin=joint.find('origin')
        pose=[float(v) for v in (origin.get('xyz')+' '+origin.get('rpy')).split()]
        assert pose==[float(v) for v in model.findtext('pose').split()]
        geometry=list(link.find('collision/geometry'))[0]
        shape=model.find('link/collision/geometry/'+geometry.tag)
        for key,value in geometry.attrib.items():
            assert [float(v) for v in value.split()]==[float(v) for v in shape.findtext(key).split()]
        assert model.findtext('static')==('false' if name=='target_cube' else 'true')
    assert world.find("model[@name='target_cube']/link/sensor/contact/topic").text=='/target_contacts'
