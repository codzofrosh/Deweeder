"""Standalone plots of simulated geometry, with no biological outcome claims."""

from pathlib import Path
import math


def plot_comparison(aggregates: list[dict], examples: list[dict], output_dir) -> list[Path]:
    """Write PNG and SVG heatmaps, interval comparisons and field overlays.

    Missing rates/intervals remain missing. Inputs are runner aggregate records
    and full geometry-kernel results optionally labelled with ``scenario``.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Patch
    from matplotlib.lines import Line2D
    import numpy as np

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    mechanisms = ("trimmer", "tine", "hoe")
    colors = {"trimmer": "#2878b5", "tine": "#c67816", "hoe": "#823ca5"}
    footnote = "SIMULATED geometry only. No plant biology or hardware validation."

    def save(fig, stem):
        fig.text(0.5, 0.012, footnote, ha="center", fontsize=9, color="#444444")
        for extension in ("png", "svg"):
            path = output_dir / f"{stem}.{extension}"
            fig.savefig(path, dpi=180, bbox_inches="tight", facecolor="white")
            paths.append(path)
        plt.close(fig)

    offsets = sorted({row["path_offset"] for row in aggregates})
    for offset in offsets:
        rows = [row for row in aggregates if row["path_offset"] == offset]
        bounds = sorted({row["lateral_bound"] for row in rows})
        speeds = sorted({row["vehicle_speed"] for row in rows})
        fig, axes = plt.subplots(3, 2, figsize=(10, 10), squeeze=False)
        for i, mechanism in enumerate(mechanisms):
            for j, (metric, title) in enumerate((
                ("weed_rate", "SIMULATED weed exposure"),
                ("crop_rate", "SIMULATED crop-envelope intersection"),
            )):
                grid = np.full((len(speeds), len(bounds)), np.nan)
                for row in rows:
                    if row["mechanism"] == mechanism and row.get(metric) is not None:
                        grid[speeds.index(row["vehicle_speed"]), bounds.index(row["lateral_bound"])] = row[metric]
                ax = axes[i, j]
                cmap = plt.get_cmap("viridis").copy()
                cmap.set_bad("#dddddd")
                im = ax.imshow(np.ma.masked_invalid(grid), vmin=0, vmax=1,
                               origin="lower", aspect="auto", cmap=cmap)
                ax.set_xticks(range(len(bounds)), [f"{b * 1000:g}" for b in bounds])
                ax.set_yticks(range(len(speeds)), [f"{v:g}" for v in speeds])
                ax.set_xlabel("Lateral error bound (mm)")
                ax.set_ylabel("Vehicle speed (m/s)")
                ax.set_title(f"{mechanism}: {title}", fontsize=10)
                for iy in range(len(speeds)):
                    for ix in range(len(bounds)):
                        value = grid[iy, ix]
                        ax.text(ix, iy, "N/A" if np.isnan(value) else f"{value:.2f}",
                                ha="center", va="center", fontsize=8,
                                color="white" if not np.isnan(value) and value < .5 else "black")
                fig.colorbar(im, ax=ax, label="Fraction", shrink=.8)
        counts = sorted({row["fixture_count"] for row in rows})
        fig.suptitle(f"Path offset {offset:g} m | fixtures per cell: {', '.join(map(str, counts))}")
        fig.text(.5, .037, "Cell labels are rounded observed fractions; 0.00 does not establish zero risk.",
                 ha="center", fontsize=9, color="#444444")
        fig.tight_layout(rect=(0, .065, 1, .97))
        save(fig, f"heatmaps_offset_{offset:g}")

    if offsets:
        fig, axes = plt.subplots(len(offsets), 2, figsize=(11, 4 * len(offsets)), squeeze=False)
        for i, offset in enumerate(offsets):
            for j, (metric, title) in enumerate((("weed", "Weed exposure"), ("crop", "Crop-envelope intersection"))):
                ax = axes[i, j]
                for mechanism in mechanisms:
                    rows = sorted((row for row in aggregates if row["path_offset"] == offset
                                   and row["mechanism"] == mechanism
                                   and math.isclose(row["vehicle_speed"], .25)),
                                  key=lambda row: row["lateral_bound"])
                    valid = [row for row in rows if row.get(f"{metric}_rate") is not None]
                    ax.plot([row["lateral_bound"] * 1000 for row in valid],
                            [row[f"{metric}_rate"] for row in valid],
                            marker="o", color=colors[mechanism], label=mechanism, alpha=.85)
                    intervals = [row for row in valid if row.get(f"{metric}_ci_low") is not None
                                 and row.get(f"{metric}_ci_high") is not None]
                    if intervals:
                        # Plot endpoints directly: bootstrap intervals need not
                        # contain the point estimate, and must not be clipped.
                        xvalues = [row["lateral_bound"] * 1000 for row in intervals]
                        lows = [row[f"{metric}_ci_low"] for row in intervals]
                        highs = [row[f"{metric}_ci_high"] for row in intervals]
                        ax.vlines(xvalues, lows, highs, color=colors[mechanism], alpha=.75)
                        ax.plot(xvalues, lows, linestyle="none", marker="_", color=colors[mechanism])
                        ax.plot(xvalues, highs, linestyle="none", marker="_", color=colors[mechanism])
                ax.set_title(f"SIMULATED {title}\nPath offset {offset:g} m; speed 0.25 m/s")
                ax.set_xlabel("Lateral error bound (mm)")
                ax.set_ylabel("Fraction with reported 95% interval")
                ax.set_ylim(-.04, 1.04)
                ax.grid(alpha=.2)
                ax.legend()
                if metric == "crop":
                    zero_bounds = [row["row_any_crop_ci_high"] for row in aggregates
                                   if row["path_offset"] == offset
                                   and math.isclose(row["vehicle_speed"], .25)
                                   and row.get("crop_rate") == 0
                                   and row.get("row_any_crop_ci_high") is not None]
                    if zero_bounds:
                        ax.text(.025, .95,
                                "Zero-event cells: upper 95% Wilson bound\n"
                                f"for any crop contact per strip = {max(zero_bounds):.3f}"
                                + (f"–{max(zero_bounds):.3f}" if max(zero_bounds) != min(zero_bounds) else ""),
                                transform=ax.transAxes, va="top", fontsize=8,
                                bbox={"facecolor": "white", "alpha": .85, "edgecolor": "none"})
        fig.text(.5, .057,
                 "Per-crop bootstrap intervals can collapse to [0, 0] after zero events; this does not establish zero risk.\n"
                 "The separately annotated strip-event Wilson bound measures a different probability. Coincident curves overlap.",
                 ha="center", fontsize=9, color="#444444")
        fig.tight_layout(rect=(0, .105, 1, 1))
        save(fig, "comparison_intervals")

    if examples:
        scenarios = list(dict.fromkeys(row.get("scenario", f"offset {row['config']['path_offset']:g} m")
                                       for row in examples))
        fig, axes = plt.subplots(len(scenarios), 3, figsize=(16, 3.6 * len(scenarios)), squeeze=False)
        for i, scenario in enumerate(scenarios):
            for j, mechanism in enumerate(mechanisms):
                ax = axes[i, j]
                matches = [row for row in examples if row["config"]["mechanism"] == mechanism
                           and row.get("scenario", f"offset {row['config']['path_offset']:g} m") == scenario]
                if not matches:
                    ax.text(.5, .5, "No example supplied", transform=ax.transAxes, ha="center")
                    ax.set_axis_off()
                    continue
                result = matches[0]
                config = result["config"]
                timeline = result["timeline"]
                active_hits = {event["plant_id"] for event in result["events"] if event["envelope"] == "active"}
                guard_hits = {event["plant_id"] for event in result["events"]
                              if event["envelope"] == "guard" and event["kind"] == "crop"}
                heading = config["robot_heading_error"]
                c, s = math.cos(heading), math.sin(heading)

                def pose(time):
                    return (-config["run_in"] + config["vehicle_speed"] * time * c,
                            config["path_offset"] + config["robot_lateral_error"]
                            + config["vehicle_speed"] * time * s)

                def sweep(start, end, radius, half_length=0):
                    a, b = pose(start), pose(end)
                    a = (a[0] - half_length * c, a[1] - half_length * s)
                    b = (b[0] + half_length * c, b[1] + half_length * s)
                    vertices = [(a[0] - radius*s, a[1] + radius*c),
                                (b[0] - radius*s, b[1] + radius*c),
                                (b[0] + radius*s, b[1] - radius*c),
                                (a[0] + radius*s, a[1] - radius*c)]
                    ax.add_patch(Polygon(vertices, color="#e5ab37", alpha=.18))
                    if mechanism != "hoe":
                        for point in (a, b):
                            ax.add_patch(Circle(point, radius, color="#e5ab37", alpha=.18))

                duration = result["summary"]["duration_s"]
                off = duration if config["command_off"] is None else config["command_off"]
                onset = config["command_on"] + config["tool_response_time"]
                end = min(duration, off + config["tool_coast_down"])
                if off > onset and onset <= end:
                    sweep(onset, end, config["tool_width"] / 2,
                          config["tool_length"] / 2 if mechanism == "hoe" else 0)
                for plant in result["plants"]:
                    hit = plant["id"] in active_hits
                    color = ("#cc3434" if hit else "#329257") if plant["kind"] == "crop" else ("#963ba8" if hit else "#999999")
                    ax.add_patch(Circle((plant["x"], plant["y"]), plant["radius"],
                                        facecolor=color, edgecolor="white", linewidth=.3, zorder=3))
                    if plant["id"] in guard_hits:
                        ax.add_patch(Circle((plant["x"], plant["y"]), plant["radius"] * 1.15,
                                            fill=False, edgecolor="#e08b00", linewidth=1.4, zorder=5))
                ax.plot([row["tool_x"] for row in timeline], [row["tool_y"] for row in timeline],
                        color="#267bb8", linewidth=1, zorder=2)
                midtime = (onset + end) / 2 if off > onset and onset <= end else duration / 2
                centre = pose(midtime)
                ax.add_patch(Circle(centre, config["guard_radius"], fill=False,
                                    linestyle="--", edgecolor="#303030", linewidth=1.1, zorder=4))
                if mechanism != "hoe":
                    ax.add_patch(Circle(centre, config["tool_active_radius"], fill=False,
                                        edgecolor="#b77c05", zorder=4))
                else:
                    vertices = [(centre[0] + x*c - y*s, centre[1] + x*s + y*c)
                                for x, y in ((-config["tool_length"]/2, -config["tool_width"]/2),
                                             (config["tool_length"]/2, -config["tool_width"]/2),
                                             (config["tool_length"]/2, config["tool_width"]/2),
                                             (-config["tool_length"]/2, config["tool_width"]/2))]
                    ax.add_patch(Polygon(vertices, fill=False, edgecolor="#b77c05", zorder=4))
                ax.set_xlim(-config["run_in"]-.1, config["row_length"]+config["run_in"]+.1)
                width = (config["row_count"]-1)*config["row_spacing"]
                ax.set_ylim(min(-.12, min(row["tool_y"] for row in timeline)-.1),
                            max(width+.12, max(row["tool_y"] for row in timeline)+.1))
                ax.set_aspect("equal", adjustable="box")
                ax.set_title(f"{scenario} | {mechanism}\nSeed {config['seed']}; actual lateral bias {config['robot_lateral_error']*1000:g} mm")
                ax.set_xlabel("Along-row position (m)")
                ax.set_ylabel("Lateral position (m)")
        legend = [Patch(color="#329257", label="Crop"), Patch(color="#cc3434", label="Crop intersection"),
                  Patch(color="#999999", label="Missed weed"), Patch(color="#963ba8", label="Exposed weed"),
                  Patch(color="#e5ab37", alpha=.3, label="Active swept zone"),
                  Line2D([], [], color="#267bb8", label="Tool trajectory"),
                  Line2D([], [], color="#303030", linestyle="--", label="Guard body disk (not protection)"),
                  Patch(facecolor="none", edgecolor="#e08b00", label="Crop contacted by guard")]
        fig.legend(handles=legend, loc="lower center", bbox_to_anchor=(.5, .035), ncol=4, fontsize=9)
        fig.suptitle("SIMULATED field geometry and continuous tool sweeps")
        fig.tight_layout(rect=(0, .13, 1, .95))
        save(fig, "field_overlays")
    return paths
