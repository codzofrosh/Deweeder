"""Standard Matplotlib figures for operation-level geometry experiments."""

import math
from pathlib import Path


def plot_operations(aggregates, examples, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Patch
    from matplotlib.lines import Line2D
    import numpy as np

    output = Path(output_dir)
    paths = []
    layouts = list(dict.fromkeys(row['layout'] for row in aggregates))
    guards = list(dict.fromkeys(row['guard'] for row in aggregates))
    bounds = sorted({row['lateral_bound'] for row in aggregates})
    modes = list(dict.fromkeys(row['error_mode'] for row in aggregates))

    def save(fig, name, evidence='SIMULATED'):
        fig.text(.5, .014, f'{evidence} geometry. Guard contact is not protection or injury. No removal or hardware validation.',
                 ha='center', fontsize=9)
        for ext in ('png', 'svg'):
            path = output / f'{name}.{ext}'
            fig.savefig(path, dpi=170, bbox_inches='tight')
            paths.append(path)
        plt.close(fig)

    for mode in modes:
        fig, axes = plt.subplots(len(guards), 3, figsize=(12, 3.8*len(guards)), squeeze=False)
        for i, guard in enumerate(guards):
            for j, (key, title) in enumerate((('weed_rate', 'Unique weed exposure'),
                                               ('crop_rate', 'Unique crop intersection'),
                                               ('guard_rate', 'Unique guard/crop contact'))):
                grid = np.full((len(layouts), len(bounds)), np.nan)
                for row in aggregates:
                    if row['guard'] == guard and row['error_mode'] == mode and row[key] is not None:
                        grid[layouts.index(row['layout']), bounds.index(row['lateral_bound'])] = row[key]
                ax = axes[i, j]
                im = ax.imshow(grid, vmin=0, vmax=1, aspect='auto', cmap='viridis')
                ax.set_xticks(range(len(bounds)), [f'{x*1000:g}' for x in bounds])
                ax.set_yticks(range(len(layouts)), layouts)
                ax.set_title(f'{guard}: {title}', fontsize=10)
                ax.set_xlabel('Lateral bias bound (mm)')
                for y in range(len(layouts)):
                    for x in range(len(bounds)):
                        value = grid[y, x]
                        ax.text(x, y, 'N/A' if np.isnan(value) else f'{value:.1%}', ha='center', va='center',
                                color='white' if value < .5 else 'black')
                fig.colorbar(im, ax=ax, shrink=.8)
        fig.suptitle(f'SIMULATED operation unions | {mode} pass errors | {aggregates[0]["fixture_count"]} independent strips/cell')
        fig.text(.5, .06, 'Zero/rounded-zero cells do not establish zero risk. Pointwise uncertainty and denominators are in aggregates.csv.',
                 ha='center', fontsize=9)
        fig.tight_layout(rect=(0, .09, 1, .95))
        save(fig, f'operation_heatmaps_{mode}')

    fig, axes = plt.subplots(1, len(modes), figsize=(6*len(modes), 4.8), squeeze=False)
    colors = plt.get_cmap('tab10')
    for i, mode in enumerate(modes):
        ax = axes[0, i]
        for j, layout in enumerate(layouts):
            row = next(r for r in aggregates if r['layout'] == layout and r['guard'] == guards[0]
                       and r['error_mode'] == mode and r['lateral_bound'] == max(bounds))
            ax.scatter(row['total_seconds'], row['weed_rate'], color=colors(j), label=layout, s=55)
            if row['weed_ci_low'] is not None:
                ax.vlines(row['total_seconds'], row['weed_ci_low'], row['weed_ci_high'], color=colors(j))
        ax.set_title(f'{mode} errors; bound {max(bounds)*1000:g} mm')
        ax.set_xlabel('Operation seconds (includes assumed reposition time)')
        ax.set_ylabel('Unique weed exposure, 95% fixture bootstrap')
        ax.set_ylim(0, 1.05)
        ax.grid(alpha=.2)
        ax.legend()
    fig.suptitle('SIMULATED coverage versus operation time; guard choice does not change active exposure')
    fig.tight_layout(rect=(0, .09, 1, .95))
    save(fig, 'coverage_vs_time')

    fig, axes = plt.subplots(len(layouts), len(guards), figsize=(12, 2.7*len(layouts)), squeeze=False)
    for i, layout in enumerate(layouts):
        for j, guard_name in enumerate(guards):
            result = next(e for e in examples if e['layout'] == layout and e['guard']['name'] == guard_name)
            ax = axes[i, j]
            active = {e['plant_id'] for e in result['events'] if e['envelope'] == 'active'}
            guard_hits = {e['plant_id'] for e in result['events'] if e['envelope'] == 'guard'}
            for p in result['plants']:
                color = ('#c33' if p['id'] in active else '#298354') if p['kind'] == 'crop' else ('#923eaa' if p['id'] in active else '#aaa')
                ax.add_patch(Circle((p['x'], p['y']), p['radius'], facecolor=color,
                                    edgecolor='#e79600' if p['kind']=='crop' and p['id'] in guard_hits else 'white',
                                    linewidth=1.5, zorder=4))
            for pas in result['passes']:
                c = pas['config']
                co, si = math.cos(c['robot_heading_error']), math.sin(c['robot_heading_error'])
                origin = (-c['run_in'], c['path_offset']+c['robot_lateral_error'])
                end = (origin[0]+c['vehicle_speed']*pas['duration_s']*co,
                       origin[1]+c['vehicle_speed']*pas['duration_s']*si)
                radius = c['tool_width']/2
                points = [(origin[0]-radius*si,origin[1]+radius*co), (end[0]-radius*si,end[1]+radius*co),
                          (end[0]+radius*si,end[1]-radius*co), (origin[0]+radius*si,origin[1]-radius*co)]
                ax.add_patch(Polygon(points,color='#edb742',alpha=.17))
                ax.plot([origin[0],end[0]],[origin[1],end[1]],color='#287caf',linewidth=.7)
                centre = ((origin[0]+end[0])/2,(origin[1]+end[1])/2)
                for comp in result['guard']['components']:
                    x=centre[0]+comp['x']*co-comp['y']*si
                    y=centre[1]+comp['x']*si+comp['y']*co
                    if comp['shape']=='disk':
                        ax.add_patch(Circle((x,y),comp['radius'],fill=False,edgecolor='#333',zorder=5))
                    else:
                        verts=[(x+dx*co-dy*si,y+dx*si+dy*co) for dx,dy in
                               [(-comp['length']/2,-comp['width']/2),(comp['length']/2,-comp['width']/2),
                                (comp['length']/2,comp['width']/2),(-comp['length']/2,comp['width']/2)]]
                        ax.add_patch(Polygon(verts,color='#333',zorder=5))
            ax.set_xlim(-.22,result['config']['row_length']+.22)
            ax.set_ylim(-.1,(result['config']['row_count']-1)*result['config']['row_spacing']+.1)
            ax.set_aspect('equal')
            ax.set_title(f'{layout} | {guard_name} | seed {result["config"]["seed"]}; bound {result["lateral_bound"]*1000:g} mm',fontsize=10)
            ax.set_xlabel('Along row (m)')
            ax.set_ylabel('Lateral (m)')
    fig.legend(handles=[Patch(color='#298354',label='Crop'),Patch(color='#c33',label='Crop intersection'),
                        Patch(color='#923eaa',label='Exposed weed'),Patch(color='#aaa',label='Missed weed'),
                        Patch(facecolor='none',edgecolor='#e79600',label='Guard/crop contact'),
                        Line2D([],[],color='#333',label='Guard components at midpoint')],
               loc='lower center',bbox_to_anchor=(.5,.045),ncol=3,fontsize=9)
    fig.suptitle('SIMULATED multi-pass layouts and dimensioned guard projections')
    fig.tight_layout(rect=(0,.11,1,.96))
    save(fig,'operation_layouts')

    fig, axes = plt.subplots(1, len(guards), figsize=(5*len(guards), 5), squeeze=False)
    for j, name in enumerate(guards):
        example = next(e for e in examples if e['guard']['name']==name)
        ax = axes[0,j]
        ax.add_patch(Circle((0,0),example['config']['tool_active_radius']*1000,
                            facecolor='#f5db9f',edgecolor='#b37a00',alpha=.7))
        for component in example['guard']['components']:
            x,y=component['x']*1000,component['y']*1000
            if component['shape']=='disk':
                ax.add_patch(Circle((x,y),component['radius']*1000,fill=False,edgecolor='#333',linewidth=2))
                label=f"{component['name']}: R={component['radius']*1000:g} mm"
            else:
                length,width=component['length']*1000,component['width']*1000
                ax.add_patch(Polygon([(x-length/2,y-width/2),(x+length/2,y-width/2),
                                      (x+length/2,y+width/2),(x-length/2,y+width/2)],color='#444'))
                label=f"{component['name']}: {length:g} x {width:g} mm at ({x:g}, {y:g})"
            ax.plot([],[],label=label,color='#444')
        ax.annotate('Forward',xy=(90,0),xytext=(55,0),arrowprops={'arrowstyle':'->'},fontsize=9)
        ax.set_xlim(-110,115); ax.set_ylim(-90,90); ax.set_aspect('equal')
        ax.set_xlabel('Tool-local X (mm)'); ax.set_ylabel('Tool-local Y (mm)')
        ax.grid(alpha=.2); ax.set_title(name); ax.legend(loc='upper center',bbox_to_anchor=(.5,-.2),fontsize=8)
    fig.suptitle('ESTIMATED guard projections around an 80 mm active disk\nConcept dimensions only; no compliant protection or manufacturing validation')
    fig.tight_layout(rect=(0,.12,1,.88))
    save(fig,'guard_projections',evidence='ESTIMATED')
    return paths
