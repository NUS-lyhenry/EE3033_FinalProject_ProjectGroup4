import math
from ee3033_project.scan_core import resample


def test_self_returns_become_unknown_not_false_clear_rays():
    ranges=resample([0.05,0.07,0.15,0.25],-math.pi,math.pi/2,0.,8.,4,0.14)
    assert math.isnan(ranges[0]) and math.isnan(ranges[1])
    assert ranges[2:]==[0.15,0.25]


def test_nearest_obstacle_survives_bin_merge():
    ranges=resample([2.,1.,3.],0.,0.001,0.,8.,450,0.14)
    assert min(r for r in ranges if math.isfinite(r))==1.


def test_self_occlusion_wins_over_another_point_in_same_bin():
    ranges=resample([0.05,1.],0.,0.001,0.,8.,450,0.14)
    assert math.isnan(ranges[225])


def test_invalid_ranges_and_empty_scan_do_not_create_obstacles():
    ranges=resample([math.nan,math.inf,-1.,20.],0.,0.5,0.,8.,450,0.14)
    assert all(not math.isfinite(r) for r in ranges)
    assert sum(math.isinf(r) for r in ranges)==1
    assert all(math.isnan(r) for r in resample([],0.,.1,0.,8.,450))


def test_default_keeps_close_external_returns_and_leaves_unobserved_bins_unknown():
    ranges=resample([.09,.13,.2],0.,.1,.08,8.,450)
    assert .09 in ranges and .13 in ranges
    assert sum(math.isnan(r) for r in ranges)==447


def test_explicit_legacy_mask_does_not_change_real_sensor_range_metadata():
    import ast
    from pathlib import Path
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/scan_resampler.py'
    tree=ast.parse(source.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    ns=dict(Node=object,math=math,struct=__import__('struct'),resample=resample,LaserScan=lambda:__import__('types').SimpleNamespace())
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
    robot=ns['ScanResampler'].__new__(ns['ScanResampler'])
    robot.p={'bins':450,'self_mask_radius':0.0}
    robot.get_parameter=lambda name:__import__('types').SimpleNamespace(value=0.)
    result=[];robot.pub=__import__('types').SimpleNamespace(publish=result.append)
    msg=__import__('types').SimpleNamespace(header=object(),range_min=.08,range_max=8.,scan_time=.1,
        angle_min=0.,angle_increment=.1,ranges=[.09,.13,.2])
    robot.on_scan(msg)
    assert result[0].range_min==.08 and .09 in result[0].ranges


def test_explicit_eight_centimetre_exclusion_preserves_boundary_and_all_farther_points():
    ranges=resample([.079,.08,.081,.13,math.inf],0.,math.pi/3,0.,8.,12,.08)
    assert math.isnan(ranges[6])
    assert ranges[8]==.08 and ranges[10]==.081 and ranges[0]==.13
    assert math.isinf(ranges[2])


def test_node_mask_uses_committed_parameter_and_float32_equality_boundary():
    import ast,struct
    from pathlib import Path
    from types import SimpleNamespace
    source=Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/scan_resampler.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,math=math,struct=struct,resample=resample,LaserScan=SimpleNamespace)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),env)
    node=env['ScanResampler'].__new__(env['ScanResampler'])
    node.p={'bins':450,'self_mask_radius':.2}  # Deliberately stale initialization cache.
    committed=[.08];node.get_parameter=lambda _:SimpleNamespace(value=committed[0])
    output=[];node.pub=SimpleNamespace(publish=output.append)
    f32=lambda value:struct.unpack('f',struct.pack('f',value))[0]
    scan=SimpleNamespace(header=object(),range_min=0.,range_max=12.,scan_time=.1,
        angle_min=0.,angle_increment=2*math.pi/450,ranges=[f32(.079),f32(.08),f32(.081)])
    node.on_scan(scan)
    assert math.isnan(output[-1].ranges[225])
    assert output[-1].ranges[226]==f32(.08) and output[-1].ranges[227]==f32(.081)
    committed[0]=0.;node.on_scan(scan)
    assert output[-1].ranges[225]==f32(.079) and output[-1].range_min==0.
