import importlib.util
from pathlib import Path

import numpy as np

spec=importlib.util.spec_from_file_location('simulation_surfaces',Path(__file__).parents[1]/'scripts/simulation_surfaces.py')
surfaces=importlib.util.module_from_spec(spec);spec.loader.exec_module(surfaces)


def top_down(x=.15):
    k=np.array([[300.,0.,320.],[0.,300.,240.],[0.,0.,1.]])
    return surfaces.surface_image([x,.15,1.],np.diag([1.,-1.,-1.]),k,[])


def test_physical_thirty_centimetre_tiles_have_ninety_pixel_spacing_at_one_metre():
    image=top_down()
    assert surfaces.TILE_SIZE_M==.30
    assert image.shape==(480,640,3) and image.dtype==np.uint8
    # At f=300 px and h=1m, 30cm spans exactly90 pixels on this plane.
    for column in (275,365,455):
        assert image[240,column].mean()+8 < image[240,320].mean()


def test_tile_pattern_moves_with_the_world_instead_of_sticking_to_the_screen():
    first=top_down();moved=top_down(.25)
    assert moved[240,335].mean()+8 < moved[240,365].mean()
    assert not np.array_equal(first,moved)
    assert np.array_equal(first,top_down())


def test_near_wall_occludes_a_farther_wall_regardless_of_input_order():
    # Optical axes x=world y, y=-world z, z=-world x.
    rotation=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
    k=np.array([[450.,0.,320.],[0.,450.,240.],[0.,0.,1.]])
    walls=np.array([[[-1.,-.7],[-1.,.7]],[[-2.,-1.],[-2.,1.]]])
    a=surfaces.surface_image([0.,0.,.22],rotation,k,walls)
    b=surfaces.surface_image([0.,0.,.22],rotation,k,walls[::-1])
    assert np.array_equal(a,b)
    assert np.std(a[120:200,200:400])>1
