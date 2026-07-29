#!/usr/bin/env python3
"""Parse an SDF model file and generate a Graphviz graph of links, joints, and frames."""

import xml.etree.ElementTree as ET
import subprocess
import sys
import os


def sdf_to_dot(sdf_path, output_name=None):
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model = root.find(".//model")
    if model is None:
        print("ERROR: No <model> element found in SDF.")
        sys.exit(1)

    model_name = model.get("name", "robot")
    if output_name is None:
        output_name = model_name

    lines = []
    lines.append(f'digraph "{model_name}" {{')
    lines.append('    rankdir=TB;')
    lines.append('    node [shape=box, style=filled, fontname="Arial"];')
    lines.append('')

    # Collect links
    links = []
    for link in model.findall("link"):
        name = link.get("name")
        links.append(name)
        lines.append(f'    "{name}" [fillcolor="#4FC3F7", label="{name}\\n(link)"];')

    lines.append('')

    # Collect frames
    for frame in model.findall("frame"):
        name = frame.get("name")
        attached = frame.get("attached_to", "???")
        lines.append(f'    "{name}" [fillcolor="#CE93D8", shape=ellipse, label="{name}\\n(frame)"];')
        lines.append(f'    "{attached}" -> "{name}" [style=dashed, color="#9C27B0", label="attached"];')

    lines.append('')

    # Collect joints
    for joint in model.findall("joint"):
        jname = joint.get("name")
        jtype = joint.get("type", "")
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = parent.text if parent is not None else "???"
        child_name = child.text if child is not None else "???"

        lines.append(f'    "{jname}" [fillcolor="#FFD54F", shape=diamond, label="{jname}\\n({jtype})"];')
        lines.append(f'    "{parent_name}" -> "{jname}" [color="#FF6F00"];')
        lines.append(f'    "{jname}" -> "{child_name}" [color="#FF6F00"];')

    lines.append('')

    # Collect sensors (show as notes on their parent link)
    for link in model.findall("link"):
        link_name = link.get("name")
        for sensor in link.findall("sensor"):
            sname = sensor.get("name")
            stype = sensor.get("type", "")
            lines.append(f'    "{sname}" [fillcolor="#A5D6A7", shape=octagon, label="{sname}\\n({stype})"];')
            lines.append(f'    "{link_name}" -> "{sname}" [style=dotted, color="#388E3C", label="sensor"];')

    lines.append('')

    # Collect plugins
    for plugin in model.findall("plugin"):
        pname = plugin.get("name", "plugin")
        short_name = pname.split("::")[-1]
        lines.append(f'    "{pname}" [fillcolor="#FFAB91", shape=hexagon, label="{short_name}\\n(plugin)"];')
        lines.append(f'    "{model_name}_root" [label="{model_name}\\n(model)", fillcolor="#E0E0E0", shape=box3d];')
        lines.append(f'    "{model_name}_root" -> "{pname}" [style=dotted, color="#BF360C", label="plugin"];')

    lines.append('')

    # Legend
    lines.append('    subgraph cluster_legend {')
    lines.append('        label="Legend";')
    lines.append('        style=filled;')
    lines.append('        fillcolor="#F5F5F5";')
    lines.append('        fontname="Arial Bold";')
    lines.append('        legend_link [fillcolor="#4FC3F7", shape=box, label="Link"];')
    lines.append('        legend_joint [fillcolor="#FFD54F", shape=diamond, label="Joint"];')
    lines.append('        legend_frame [fillcolor="#CE93D8", shape=ellipse, label="Frame"];')
    lines.append('        legend_sensor [fillcolor="#A5D6A7", shape=octagon, label="Sensor"];')
    lines.append('        legend_plugin [fillcolor="#FFAB91", shape=hexagon, label="Plugin"];')
    lines.append('        legend_link -> legend_joint -> legend_frame -> legend_sensor -> legend_plugin [style=invis];')
    lines.append('    }')

    lines.append("}")

    dot_content = "\n".join(lines)

    gv_file = f"{output_name}.gv"
    pdf_file = f"{output_name}.pdf"
    png_file = f"{output_name}.png"

    with open(gv_file, "w") as f:
        f.write(dot_content)

    # Try to generate PDF and PNG
    try:
        subprocess.run(["dot", "-Tpdf", gv_file, "-o", pdf_file], check=True)
        subprocess.run(["dot", "-Tpng", gv_file, "-o", png_file], check=True)
        print(f"Generated: {gv_file}, {pdf_file}, {png_file}")
    except FileNotFoundError:
        print(f"Graphviz 'dot' not found. Install with: sudo apt install graphviz")
        print(f"Generated: {gv_file} (render manually with: dot -Tpng {gv_file} -o {png_file})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <model.sdf> [output_name]")
        sys.exit(1)

    sdf_path = sys.argv[1]
    output_name = sys.argv[2] if len(sys.argv) > 2 else None
    sdf_to_dot(sdf_path, output_name)
