#include "mapping.h"
#include "feature_object.h"

void Mapping::Do(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	for (size_t i = 0; i < object2.m_cloud->size(); ++i)
	{
		std::vector<int> two2oneindices(1);
		std::vector<float> two2onedistances(1);
		object1.m_feature_tree->nearestKSearch(*object2.m_features, (int)i, 1, two2oneindices, two2onedistances);
		
		if (two2onedistances[0] < 2.0f)
		{
			indices1.push_back(two2oneindices[0]);
			indices2.push_back(i);
		}
	}
}

void Mapping::DoLowe(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	//unsigned nsize = 2;
	//for (size_t i = 0; i < object2.m_d_cloud->size(); ++i)
	//{
	//	if (object2.m_inner_index[i])
	//	{
	//		std::vector<int> two2oneindices(nsize);
	//		std::vector<float> two2onedistances(nsize);
	//		object1.m_feature_tree->nearestKSearch(*object2.m_d_features, (int)i, nsize, two2oneindices, two2onedistances);
	//
	//		if (object1.m_inner_index[two2oneindices[0]] && (two2onedistances[0] / two2onedistances[1] < 0.5f))
	//		{
	//			indices1.push_back(two2oneindices[0]);
	//			indices2.push_back(i);
	//		}
	//	}
	//}
	//
	//std::cout << "Find correspondences ." << indices1.size() << std::endl;
}
