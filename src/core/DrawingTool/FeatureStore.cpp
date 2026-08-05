#include "FeatureStore.h"

FeatureId FeatureStore::addFeature(osgEarth::Feature* feature)
{
	if (feature == nullptr) {
		return 0;
	}

	const FeatureId id = m_nextId++;
	m_features[id] = feature;
	return id;
}

bool FeatureStore::removeFeature(FeatureId id)
{
	return m_features.erase(id) > 0;
}

osgEarth::Feature* FeatureStore::feature(FeatureId id)
{
	const auto it = m_features.find(id);
	if (it == m_features.end()) {
		return nullptr;
	}
	return it->second.get();
}

void FeatureStore::clear()
{
	m_features.clear();
}

std::vector<FeatureId> FeatureStore::ids() const
{
	std::vector<FeatureId> result;
	result.reserve(m_features.size());

	for (const auto& entry : m_features) {
		result.push_back(entry.first);
	}

	return result;
}
