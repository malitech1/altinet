# Floorplan to dashboard workflow

This document describes the workflow for taking an interactive floorplan that
was created in the Altinet web builder and turning it into the 3D model that is
shown on the authenticated dashboard home page.

## 1. Sketch the plan in the builder

1. Sign in to the web UI and open the **Builder**.
2. Use the wall and room tools to sketch each storey of the property. The
   canvas now supports multiple levels – add, duplicate or delete storeys from
   the header and rename them in the sidebar.
3. Each level is saved locally in your browser (persisted in `localStorage`).
   When you're finished, click **Export plan for Blender**. A JSON file will be
   downloaded with every wall, room and level as well as the grid metadata
   needed by Blender.

## 2. Automatic 3D model generation

The builder now syncs every change to the Django backend. The API writes the
latest floorplan JSON to `assets/floorplans/latest.json` and regenerates the
OBJ served to the dashboard (`backend/web/static/web/models/home.obj`) using a
lightweight Python renderer. Reload the authenticated home page to see the
updated geometry moments after saving in the builder.

### Optional: manual Blender exports

If you need to iterate with Blender directly, use **Export plan for Blender**
and follow the original workflow:

```bash
blender --background --python scripts/generate_floorplan.py -- \
    --plan assets/floorplans/latest.json \
    --out assets/floorplans/latest.blend \
    --obj-out backend/web/static/web/models/home.obj
```

## 3. Review the model in the dashboard

1. Reload the authenticated home page. The viewer automatically references the
   OBJ path defined by the `ALTINET_HOME_MODEL` environment variable (default:
   `web/models/home.obj`).
2. Orbit, pan and zoom around the model to verify the geometry matches the
   exported floorplan. When you iterate on the plan, repeat the export + Blender
   step to update the OBJ.

## Tips

- The CLI accepts `--storey-height`, `--wall-height` and `--floor-thickness` to
  tweak proportions per project.
- If a plan export is missing `unitScale`, pass `--unit-scale` to define how
  many metres each grid step represents.
- You can keep multiple OBJ exports and switch between them at runtime by
  setting `ALTINET_HOME_MODEL=/static/path/to/model.obj` before starting Django.
