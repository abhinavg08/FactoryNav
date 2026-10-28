"""Generate the virtual desktop's initial window layout from Openbox defaults."""
from pathlib import Path
import xml.etree.ElementTree as ET

tree = ET.parse('/etc/xdg/openbox/rc.xml')
root = tree.getroot()
namespace = root.tag.split('}')[0] + '}'
applications = root.find(f'{namespace}applications')
for title, x in [('Gazebo Sim', 0), ('*RViz*', 800)]:
    application = ET.SubElement(applications, f'{namespace}application', title=title)
    position = ET.SubElement(application, f'{namespace}position', force='yes')
    ET.SubElement(position, f'{namespace}x').text = str(x)
    ET.SubElement(position, f'{namespace}y').text = '0'
    size = ET.SubElement(application, f'{namespace}size')
    ET.SubElement(size, f'{namespace}width').text = '800'
    ET.SubElement(size, f'{namespace}height').text = '875'
    ET.SubElement(application, f'{namespace}maximized').text = 'no'
output = Path('/tmp/factorynav_openbox.xml')
tree.write(output, encoding='utf-8', xml_declaration=True)
print(output)
