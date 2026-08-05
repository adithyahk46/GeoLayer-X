#ifndef FEATURESTORE_H
#define FEATURESTORE_H

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <osg/ref_ptr>
#include <osgEarth/Feature>

using FeatureId = std::uint64_t;
#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT FeatureStore
{
public:
    FeatureId addFeature(
        osgEarth::Feature* feature);

    bool removeFeature(FeatureId id);

    osgEarth::Feature*
    feature(FeatureId id);

    void clear();

    std::vector<FeatureId> ids() const;

private:
    std::unordered_map<
        FeatureId,
        osg::ref_ptr<osgEarth::Feature>
    > m_features;

    FeatureId m_nextId = 1;
};

#endif // FEATURESTORE_H