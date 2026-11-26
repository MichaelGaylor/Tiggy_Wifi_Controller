"""Writes the KiCad project file: design rules and net classes."""
import json
import os

import design
import gen_sch


def build():
    classes = []
    patterns = []
    for name, width, clearance, via, drill, nets in design.NETCLASSES:
        classes.append({
            "bus_width": 12, "clearance": clearance, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
            "diff_pair_width": 0.2, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.1, "name": name,
            "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": -1 if name == "Default" else len(classes) - 1,
            "schematic_color": "rgba(0, 0, 0, 0.000)", "track_width": width, "tuning_profile": "",
            "via_diameter": via, "via_drill": drill, "wire_width": 6})
        # a supply's name stands alone; a net named on one page carries that page's name in front
        for n in nets:
            patterns += [{"netclass": name, "pattern": n}, {"netclass": name, "pattern": "/*/" + n}]
    return {
        "board": {"3dviewports": [], "design_settings": {
            "defaults": {"zones": {"min_clearance": 0.3}},
            "rules": {"max_error": 0.005, "min_clearance": 0.2, "min_connection": 0.0, "min_copper_edge_clearance": 0.3,
                      "min_hole_clearance": 0.25, "min_hole_to_hole": 0.25, "min_microvia_diameter": 0.2,
                      "min_microvia_drill": 0.1, "min_resolved_spokes": 1, "min_silk_clearance": 0.0,
                      "min_text_height": 0.6, "min_text_thickness": 0.08, "min_through_hole_diameter": 0.3,
                      "min_track_width": 0.2, "min_via_annular_width": 0.15, "min_via_diameter": 0.5,
                      "solder_mask_to_copper_clearance": 0.0, "use_height_for_length_calcs": True},
            "track_widths": [0.0, 0.25, 0.5, 1.2], "via_dimensions": [{"diameter": 0.0, "drill": 0.0}],
        }, "ipc2581": {}, "layer_pairs": [], "layer_presets": [], "viewports": []},
        "boards": [],
        "cvpcb": {"equivalence_files": []},
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": design.NAME + ".kicad_pro", "version": 3},
        "net_settings": {"classes": classes, "meta": {"version": 5}, "net_colors": None, "netclass_assignments": None,
                         "netclass_patterns": patterns},
        "pcbnew": {"last_paths": {}, "page_layout_descr_file": ""},
        # beside each name that leaves a page: the pages it is also on
        "schematic": {"drawing": {"intersheets_ref_own_page": False, "intersheets_ref_prefix": "page ",
                                  "intersheets_ref_short": False, "intersheets_ref_show": True,
                                  "intersheets_ref_suffix": ""},
                      "legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[str(gen_sch.ROOT), "Root"]] + [[str(gen_sch.sheet_uuid(key)), title]
                                                   for key, title, *_ in gen_sch.PAGES[1:]],
        "text_variables": {},
    }


if __name__ == "__main__":
    with open(os.path.join(gen_sch.PROJECT, design.NAME + ".kicad_pro"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(build(), f, indent=2)
        f.write("\n")
    print("project file written")
