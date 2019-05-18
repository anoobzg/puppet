#pragma once
#include "harmonic_caculator.h"

__declspec(dllexport) bool GetHarmonicArray(Mesh& mesh, const std::vector<unsigned>& base, const std::vector<unsigned>& neg, const std::vector<unsigned>& pos, float* harmonic)
{
	HarmonicCaculator calculator(mesh);
	calculator.Do(base, neg, pos, harmonic);

	return true;
}