#pragma once
#include <vector>

class FeatureObject;
class Mapping
{
public:
	static void Do(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2);
	static void DoLowe(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2);
	static void DoStep(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2);
	static void DoISS(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2);
};
