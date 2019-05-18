#pragma once
#ifndef _POLYHEDRON_TAG
#define _POLYHEDRON_TAG

typedef enum key_region_tag
{
	e_none_region,
	e_key_region_ao,
	e_key_region_to,
	e_key_region_1,
	e_key_region_2,
	e_key_region_3,
	e_key_region_4,
	e_key_region_5,
	e_key_region_6,
	e_key_region_7,
	e_key_region_8
} KRTag;

static double keyregion_smooth_coff[10] = {0.0303, 0.98, 0.98, 0.80, 0.62, 0.40, 0.22, 0.15, 0.11, 0.05 };
#endif // _POLYHEDRON_TAG