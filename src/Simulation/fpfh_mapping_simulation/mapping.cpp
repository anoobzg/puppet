#include "mapping.h"
#include "feature_object.h"

void Mapping::Do(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	for (size_t i = 0; i < object2.m_d_cloud->size(); ++i)
	{
		if (object2.m_inner_index[i])
		{
			std::vector<int> two2oneindices(1);
			std::vector<float> two2onedistances(1);
			object1.m_feature_tree->nearestKSearch(*object2.m_d_features, (int)i, 1, two2oneindices, two2onedistances);
			
			if (two2onedistances[0] < 2.0f && object1.m_inner_index[two2oneindices[0]])
			{
				indices1.push_back(two2oneindices[0]);
				indices2.push_back(i);
			}
		}
	}
}

void Mapping::DoLowe(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	unsigned nsize = 2;
	for (size_t i = 0; i < object2.m_d_cloud->size(); ++i)
	{
		if (object2.m_inner_index[i])
		{
			std::vector<int> two2oneindices(nsize);
			std::vector<float> two2onedistances(nsize);
			object1.m_feature_tree->nearestKSearch(*object2.m_d_features, (int)i, nsize, two2oneindices, two2onedistances);

			if (object1.m_inner_index[two2oneindices[0]] && (two2onedistances[0] / two2onedistances[1] < 0.5f))
			{
				indices1.push_back(two2oneindices[0]);
				indices2.push_back(i);
			}
		}
	}

	std::cout << "Find correspondences ." << indices1.size() << std::endl;
}

void Mapping::DoStep(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	unsigned count = 0;
	unsigned neighbor_size = 20;

	static unsigned current_index = 0;
	std::vector<bool>& inner_index = object2.m_inner_index;
	unsigned total_size = (unsigned)inner_index.size();
	for (unsigned i = current_index; i < total_size; ++i)
	{
		if (inner_index[i])
		{
			std::vector<int> two_map_one(neighbor_size);
			std::vector<float> two_distance_one(neighbor_size);
			object1.m_feature_tree->nearestKSearch(*object2.m_d_features, (int)i, neighbor_size, two_map_one, two_distance_one);

			for (size_t j = 0; j < neighbor_size; ++j)
			{
				indices1.push_back(two_map_one[j]);
				indices2.push_back(i);
			}

			current_index = i + 1;
			break;
		}
	}
}

void Mapping::DoISS(FeatureObject& object1, FeatureObject& object2, std::vector<unsigned>& indices1, std::vector<unsigned>& indices2)
{
	for (size_t i = 0; i < object2.m_iss_tree_index->size(); ++i)
	{
		int index = object2.m_iss_tree_index->at(i);

		std::vector<int> two2oneindices(2);
		std::vector<float> two2onedistances(2);
		object1.m_iss_feature_tree->nearestKSearch(*object2.m_d_features, (int)index, 2, two2oneindices, two2onedistances);

		int opposite_index = two2oneindices[0];

		if (two2onedistances[0] / two2onedistances[1] < 0.8f)
		{
			indices2.push_back(index);
			indices1.push_back(opposite_index);
		}
	}
}