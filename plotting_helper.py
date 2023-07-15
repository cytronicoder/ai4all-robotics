import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

def plot_road(xmin=0.0, xmax=100.0, ymin=-12.0, ymax=12.0):
    fig, ax = plt.subplots()

    codes = [Path.MOVETO, Path.LINETO,
             Path.LINETO, Path.LINETO, Path.CLOSEPOLY]

    # Bottom grass
    verts_bottom_grass = [(xmin, ymin), (xmin, -10.0),
                          (xmax, -10.0), (xmax, ymin), (0.0, 0.0)]
    path_bottom_grass = Path(verts_bottom_grass, codes)
    patch_bottom_grass = patches.PathPatch(
        path_bottom_grass, facecolor='green', lw=2)
    ax.add_patch(patch_bottom_grass)
    # Top grass
    verts_top_grass = [(xmin, 10.0), (xmin, ymax),
                       (xmax, ymax), (xmax, 10.0), (0.0, 0.0)]
    path_top_grass = Path(verts_top_grass, codes)
    patch_top_grass = patches.PathPatch(
        path_top_grass, facecolor='green', lw=2)
    ax.add_patch(patch_top_grass)
    # Road
    verts_road = [(xmin, -10.0), (xmin, 10.0),
                  (xmax, 10.0), (xmax, -10.0), (0.0, 0.0)]
    path_road = Path(verts_road, codes)
    patch_road = patches.PathPatch(path_road, facecolor='gray', lw=2)
    ax.add_patch(patch_road)

    # Centerline
    ax.plot([xmin, xmax], [0.0, 0.0], 'w--')

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)

    fig.set_figwidth(10)
    fig.set_figheight(2)

    return fig, ax
