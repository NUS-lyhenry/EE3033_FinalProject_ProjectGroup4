"""Bounded path planning makes progress without exposing partial safety fields."""
import threading

import numpy as np
import pytest
from ee3033_project.coverage import CoveragePlanner


def planner(size=42):return CoveragePlanner(np.zeros((size,size)),.05,(0.,0.,0.),clearance=.1)


class Clock:
    def __init__(self):self.value=0.
    def __call__(self):self.value+=.001;return self.value


def pause_field(p,clock,x=1.,y=1.):
    with pytest.raises(TimeoutError):p.travel_field(x,y,deadline=clock.value+.003)
    assert p.path_pending is not None and p.path_cache is None


def test_three_finite_slices_resume_to_identical_complete_field(monkeypatch):
    reference=planner();expected,parents=reference.travel_field(1.,1.)
    p=planner();clock=Clock();monkeypatch.setattr('ee3033_project.coverage.time.monotonic',clock)
    counts=[]
    for _ in range(2):
        with pytest.raises(TimeoutError):p.travel_field(1.,1.,deadline=clock.value+.005)
        counts.append(p.path_pending['count'])
        assert p.path_cache is None
    assert 0<counts[0]<counts[1]
    actual,actual_parents=p.travel_field(1.,1.,deadline=clock.value+.005)
    assert np.array_equal(actual,expected) and actual_parents==parents
    assert p.path_pending is None and p.path_cache[1] is actual
    # A complete cache is reused without touching the heap.
    monkeypatch.setattr('ee3033_project.coverage.heapq.heappop',lambda *a:pytest.fail('Cache recomputed'))
    assert p.travel_field(1.,1.)[0] is actual


def test_cancel_invalidates_partial_and_complete_fields(monkeypatch):
    p=planner();clock=Clock();monkeypatch.setattr('ee3033_project.coverage.time.monotonic',clock)
    pause_field(p,clock)
    generation=p.path_generation;cancel=threading.Event();cancel.set()
    with pytest.raises(RuntimeError,match='canceled'):p.travel_field(1.,1.,cancel=cancel)
    assert p.path_pending is p.path_cache is None and p.path_generation>generation
    cancel.clear();p.travel_field(1.,1.)
    assert p.path_cache is not None
    cancel.set()
    with pytest.raises(RuntimeError):p.travel_field(1.,1.,cancel=cancel)
    assert p.path_cache is None


def test_new_start_discards_old_unfinished_heap(monkeypatch):
    p=planner();clock=Clock();monkeypatch.setattr('ee3033_project.coverage.time.monotonic',clock)
    pause_field(p,clock);previous=p.path_pending
    pause_field(p,clock,1.5,1.5)
    assert p.path_pending is not previous
    assert p.path_pending['start']==p.world_to_cell(1.5,1.5)
    assert p.path_pending['parents'] is not previous['parents']


def test_explicit_graph_invalidation_discards_partial_and_complete_caches(monkeypatch):
    p=planner();clock=Clock();monkeypatch.setattr('ee3033_project.coverage.time.monotonic',clock)
    pause_field(p,clock);generation=p.path_generation
    p.invalidate_paths()
    assert p.path_cache is p.path_pending is None and p.path_generation==generation+1
    p.travel_field(1.,1.);p.invalidate_paths()
    assert p.path_cache is None


def test_native_edge_mutation_invalidates_and_recomputes_shorter_path():
    p=planner();a=(.525,.525);b=(1.525,1.525)
    old=p.travel_field(*a)[0][p.world_to_cell(*b)]
    p.components=np.zeros(p.grid.shape,dtype=int);p.reachable=np.zeros(p.grid.shape,dtype=bool)
    assert p.accept_native_connection(1,[a,b])
    assert p.path_cache is p.path_pending is None
    new=p.travel_field(*a)[0][p.world_to_cell(*b)]
    assert new==pytest.approx(2**.5) and new<old


def test_prepare_invalidates_old_graph_work(monkeypatch):
    p=planner();p.travel_field(1.,1.);generation=p.path_generation
    monkeypatch.setattr(p,'visible',lambda *a,**k:frozenset())
    with pytest.raises(ValueError,match='viewpoints'):
        p.prepare(1.,1.)
    assert p.path_cache is p.path_pending is None and p.path_generation>generation


def test_busy_worker_does_not_wait_or_consume_deadline_clock(monkeypatch):
    p=planner();held=threading.Event();release=threading.Event()
    def owner():
        with p.path_lock:
            held.set();release.wait(timeout=1.)
    thread=threading.Thread(target=owner);thread.start();assert held.wait(timeout=1.)
    monkeypatch.setattr('ee3033_project.coverage.time.monotonic',lambda:pytest.fail('Busy waited on deadline'))
    try:
        with pytest.raises(TimeoutError,match='in progress'):p.travel_field(1.,1.,deadline=1.)
        with pytest.raises(TimeoutError,match='busy'):p.invalidate_paths()
        assert thread.is_alive()
    finally:
        release.set();thread.join(timeout=1.)


def test_cancellation_during_expansion_discards_pending_work():
    class Cancel:
        def __init__(self):self.checks=0
        def is_set(self):
            self.checks+=1
            return self.checks>=3
    p=planner()
    with pytest.raises(RuntimeError,match='canceled'):p.travel_field(1.,1.,cancel=Cancel())
    assert p.path_pending is p.path_cache is None
    assert not p.path_lock.locked()
