"""Analyze one recorded multi-pass cell with verified saved field/event data."""

import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path

from simulation.geometry.config import Config
from simulation.geometry.field import Plant
from simulation.geometry.misses import classify_misses, summarize_misses
from simulation.geometry.export import _csv_text
from .statistics import summarize


def analyze(input_dir, output_dir, layout='inset_four', guard='open_front_u',
            mode='independent', bound=.03, clearance=.03, make_plots=True):
    source=Path(input_dir); output=Path(output_dir)
    metadata=json.loads((source/'metadata.json').read_text(encoding='utf-8'))
    for name in ('fields.json','rows.csv','events.csv.gz'):
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=metadata['artifact_sha256'][name]:
            raise ValueError(f'input artifact hash mismatch: {name}')
    with (source/'rows.csv').open(encoding='utf-8',newline='') as stream:
        selected=[row for row in csv.DictReader(stream) if row['layout']==layout and row['guard']==guard
                  and row['error_mode']==mode and float(row['lateral_bound'])==bound]
    if not selected or len({r['cell_id'] for r in selected})!=1 or len({r['seed'] for r in selected})!=len(selected):
        raise ValueError('selector must identify one complete cell with unique seeds')
    if len(selected)!=metadata['independent_strip_count']:
        raise ValueError('cell has incomplete fixture count')
    cell=selected[0]['cell_id']
    rows={int(r['seed']):r for r in selected}
    events={seed:[] for seed in rows}
    with gzip.open(source/'events.csv.gz','rt',encoding='utf-8',newline='') as stream:
        for event in csv.DictReader(stream):
            if event['cell_id']==cell:
                events[int(event['seed'])].append(event)
    fields=json.loads((source/'fields.json').read_text(encoding='utf-8'))
    if len({f['seed'] for f in fields})!=len(fields):
        raise ValueError('duplicate field seeds')
    config=Config.from_mapping(metadata['base_config'])
    records=[]; fixture_records=[]
    for field in fields:
        seed=field['seed']
        if seed not in rows: continue
        plants=[Plant(**p) for p in field['plants']]
        result=classify_misses(config,plants,events[seed],metadata['plan']['layouts'][layout],clearance)
        summary=summarize_misses(result)
        if summary['counts']['exposed']!=int(rows[seed]['weed_exposures']) or summary['target_count']!=int(rows[seed]['weed_count']):
            raise ValueError('exposure/target counts differ from saved operation')
        records.extend({**r,'seed':seed} for r in result)
        fixture_records.append({'seed':seed,**summary})
    if len(fixture_records)!=len(rows): raise ValueError('missing field data')
    pooled={category:sum(r['counts'][category] for r in fixture_records)
            for category in ('exposed','near_crop','nominal_gap','path_error')}
    targets=sum(r['target_count'] for r in fixture_records)
    uncertainty={}
    for category in pooled:
        samples=[{'crop_count':0,'crop_intersections':0,'guard_contacts':0,
                  'weed_count':r['target_count'],'weed_exposures':r['counts'][category]} for r in fixture_records]
        stats=summarize(samples,metadata['plan']['bootstrap_samples'],metadata['plan']['bootstrap_seed'])
        uncertainty[category]={'fraction':stats['weed_rate'],'ci_low':stats['weed_ci_low'],
                               'ci_high':stats['weed_ci_high'],'status':stats['ci_status']['weed']}
    result={'evidence_class':'SIMULATED','input_metadata_sha256':hashlib.sha256((source/'metadata.json').read_bytes()).hexdigest(),
            'selector':{'layout':layout,'guard':guard,'mode':mode,'bound':bound},
            'crop_clearance_threshold_m':clearance,'independent_strips':len(rows),'target_count':targets,
            'counts':pooled,'uncertainty':uncertainty,
            'nominal_exposed_count':sum(r['nominal_exposed'] for r in records),
            'lost_nominal_exposure_count':sum(r['nominal_exposed'] and not r['exposed'] for r in records),
            'gained_outside_nominal_count':sum(r['exposed'] and not r['nominal_exposed'] for r in records),
            'exposed_near_crop_count':sum(r['exposed_near_crop_count'] for r in fixture_records),
            'assumptions':['near_crop is a diagnostic threshold, not a validated safe distance',
                           'classification precedence: exposed, missed near_crop, nominal_gap or path_error',
                           'path_error means loss relative to ideal zero-error always-active nominal passes',
                           'no physical safety or removal inference; intervals resample entire strips'],
            'source_sha256':{str(p.relative_to(Path(__file__).resolve().parents[1])).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'geometry'/'misses.py')}}
    output.mkdir(parents=True,exist_ok=True)
    (output/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n',encoding='utf-8')
    (output/'targets.csv').write_text(_csv_text(records,tuple(records[0]) if records else ('id','category','seed')),encoding='utf-8',newline='')
    (output/'fixtures.csv').write_text(_csv_text(fixture_records,tuple(fixture_records[0])),encoding='utf-8',newline='')
    if make_plots: plot(result,records,output)
    return result


def plot(result,records,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    labels={'exposed':'Exposed','near_crop':'Missed: near crop','nominal_gap':'Missed: nominal layout gap','path_error':'Missed: loss from nominal pass'}
    colors=['#2c8754','#c78420','#595cb6','#b04858']
    fig,axes=plt.subplots(1,2,figsize=(12,4.8))
    keys=list(labels)
    values=[result['uncertainty'][k]['fraction'] for k in keys]
    if not records:
        for ax in axes:
            ax.text(.5,.5,'No target weeds; rates unavailable',ha='center',transform=ax.transAxes)
            ax.set_axis_off()
        fig.suptitle('SIMULATED target accounting: empty target population')
        fig.savefig(output/'miss_breakdown.png',dpi=170,bbox_inches='tight');plt.close(fig)
        return
    axes[0].barh([labels[k] for k in keys],values,color=colors)
    for i,k in enumerate(keys):
        u=result['uncertainty'][k]
        if u['ci_low'] is not None:axes[0].hlines(i,u['ci_low'],u['ci_high'],color='black',linewidth=2)
        axes[0].text(values[i]+.015,i,f'{values[i]:.1%}',va='center')
    axes[0].set_xlim(0,1);axes[0].set_xlabel('Fraction of all target weeds; 95% strip bootstrap')
    axes[0].invert_yaxis()
    seed=min(r['seed'] for r in records)
    for k,color in zip(keys,colors):
        data=[r for r in records if r['seed']==seed and r['category']==k]
        axes[1].scatter([r['x'] for r in data],[r['y'] for r in data],s=24,color=color,label=labels[k])
    axes[1].set_xlabel('Along-row position (m)');axes[1].set_ylabel('Lateral position (m)')
    axes[1].set_title(f'Illustrative target locations, seed {seed}')
    axes[1].legend(fontsize=8,loc='upper left',bbox_to_anchor=(1.02,1))
    fig.suptitle(f'SIMULATED {result["selector"]["layout"]} target accounting: every weed counted once')
    fig.text(.5,.015,f'Near-crop bin is an unvalidated {result["crop_clearance_threshold_m"]*1000:g} mm surface-clearance hypothesis. No removal or crop-safety claim.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.07,1,.94))
    fig.savefig(output/'miss_breakdown.png',dpi=170,bbox_inches='tight');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--layout',default='inset_four');parser.add_argument('--guard',default='open_front_u')
    parser.add_argument('--mode',default='independent');parser.add_argument('--bound',type=float,default=.03)
    parser.add_argument('--clearance',type=float,default=.03);parser.add_argument('--no-plots',action='store_true')
    args=parser.parse_args()
    try:
        result=analyze(args.input,args.output,args.layout,args.guard,args.mode,args.bound,args.clearance,not args.no_plots)
    except (ValueError,KeyError,OSError,ImportError) as exc:parser.exit(2,f'Miss analysis failed: {exc}\n')
    print(json.dumps({'targets':result['target_count'],'counts':result['counts']},indent=2))


if __name__=='__main__':main()
