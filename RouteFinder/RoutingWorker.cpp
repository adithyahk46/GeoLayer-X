#include "RoutingWorker.h"
#include <ogrsf_frmts.h>
#include <QFileInfo>
#include <QDateTime>
#include <QFile>
#include <QDataStream>
#include <QTextStream>
#include <QDir>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <QtMath>               // for qDegreesToRadians, qSin, qCos, etc.
#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>

// Haversine‐based geodetic distance (meters)
static double geodeticDistance(double lon1, double lat1, double lon2, double lat2) {
    static const double R = 6371000.0; // earth radius in meters
    double dLat = qDegreesToRadians(lat2 - lat1);
    double dLon = qDegreesToRadians(lon2 - lon1);
    double a = qSin(dLat/2) * qSin(dLat/2)
            + qCos(qDegreesToRadians(lat1)) * qCos(qDegreesToRadians(lat2))
            * qSin(dLon/2) * qSin(dLon/2);
    double c = 2 * qAtan2(qSqrt(a), qSqrt(1 - a));
    return R * c;
}

// Hash for PtKey
namespace std {
template<> struct hash<RoutingWorker::PtKey> {
    size_t operator()(RoutingWorker::PtKey const& p) const noexcept {
        auto h1 = std::hash<long long>()(std::llround(p.x*1e6));
        auto h2 = std::hash<long long>()(std::llround(p.y*1e6));
        return h1 ^ (h2<<1);
    }
};
}

RoutingWorker::RoutingWorker(const QString &shapefile,
                             double sx,double sy,
                             double ex,double ey,
                             AlgorithmType algo)
    : inputPath(shapefile)
    , startX(sx), startY(sy)
    , endX(ex),   endY(ey)
    , algoType(algo)
{}

void RoutingWorker::process()
{
    emit stageUpdate("Checking graph cache...");
    QFileInfo fi(inputPath);
    QString cachePath = fi.absolutePath()
            + "/" + fi.completeBaseName() + ".graphcache";
    qint64 srcTs = fi.lastModified().toMSecsSinceEpoch();

    std::vector<PtKey> nodes;
    std::vector<std::vector<Edge>> adj;

    if (QFile::exists(cachePath)) {
        QFile cache(cachePath);
        if (cache.open(QIODevice::ReadOnly)) {
            QDataStream in(&cache);
            qint64 savedTs;
            in >> savedTs;
            if (savedTs == srcTs) {
                emit stageUpdate("Loading graph from cache...");
                int nNodes;
                in >> nNodes;
                nodes.resize(nNodes);
                for (int i = 0; i < nNodes; ++i)
                    in >> nodes[i].x >> nodes[i].y;
                adj.resize(nNodes);
                for (int u = 0; u < nNodes; ++u) {
                    int deg;
                    in >> deg;
                    adj[u].resize(deg);
                    for (int j = 0; j < deg; ++j)
                        in >> adj[u][j].to >> adj[u][j].cost;
                }
                cache.close();
            } else {
                cache.close();
                QFile::remove(cachePath);
                emit stageUpdate("Cache outdated; rebuilding graph...");
                goto BUILD_GRAPH;
            }
        } else {
            emit stageUpdate("Cannot open cache; rebuilding graph...");
            goto BUILD_GRAPH;
        }
    } else {
BUILD_GRAPH:
        emit stageUpdate("Registering GDAL drivers...");
        GDALAllRegister();

        emit stageUpdate("Opening shapefile...");
        GDALDataset *inDS = (GDALDataset*)GDALOpenEx(
                    inputPath.toStdString().c_str(),
                    GDAL_OF_VECTOR, nullptr, nullptr, nullptr
                    );
        if (!inDS) {
            emit stageUpdate("ERROR: cannot open shapefile.");
            emit finished();
            return;
        }
        OGRLayer *layer = inDS->GetLayer(0);
        if (!layer || wkbFlatten(layer->GetGeomType())!=wkbLineString) {
            emit stageUpdate("ERROR: invalid geometry.");
            GDALClose(inDS);
            emit finished();
            return;
        }

        emit stageUpdate("Building graph from shapefile...");
        std::unordered_map<PtKey,int> nodeMap;
        layer->ResetReading();
        OGRFeature *feat;
        while ((feat = layer->GetNextFeature())!=nullptr) {
            OGRLineString *line = feat->GetGeometryRef()->toLineString();
            int nPt = line->getNumPoints();
            for (int i = 0; i+1 < nPt; ++i) {
                PtKey a{ line->getX(i),   line->getY(i)   };
                PtKey b{ line->getX(i+1), line->getY(i+1) };
                int ia, ib;
                auto itA = nodeMap.find(a);
                if (itA==nodeMap.end()) {
                    ia = nodes.size();
                    nodeMap[a] = ia;
                    nodes.push_back(a);
                    adj.emplace_back();
                } else ia = itA->second;
                auto itB = nodeMap.find(b);
                if (itB==nodeMap.end()) {
                    ib = nodes.size();
                    nodeMap[b] = ib;
                    nodes.push_back(b);
                    adj.emplace_back();
                } else ib = itB->second;

                double cost = euclid(a.x,a.y, b.x,b.y);
                adj[ia].push_back({ib,cost});
                adj[ib].push_back({ia,cost});
            }
            OGRFeature::DestroyFeature(feat);
        }
        GDALClose(inDS);
        emit stageUpdate(QString("Graph built: %1 nodes").arg(nodes.size()));

        // save to cache
        QFile cache(cachePath);
        if (cache.open(QIODevice::WriteOnly)) {
            emit stageUpdate("Saving graph cache...");
            QDataStream out(&cache);
            out << srcTs;
            int nNodes = nodes.size();
            out << nNodes;
            for (auto &n : nodes)
                out << n.x << n.y;
            for (int u = 0; u < nNodes; ++u) {
                int deg = adj[u].size();
                out << deg;
                for (auto &e : adj[u])
                    out << e.to << e.cost;
            }
            cache.close();
        } else {
            emit stageUpdate("WARNING: failed to write cache.");
        }
    }

    // snap start/end
    emit stageUpdate("Snapping start/end...");
    int sN=0,eN=0;
    double bestS=std::numeric_limits<double>::infinity(),
            bestE=std::numeric_limits<double>::infinity();
    for (int i=0; i<nodes.size(); ++i) {
        double ds=euclid(startX,startY,nodes[i].x,nodes[i].y);
        if (ds<bestS){ bestS=ds; sN=i; }
        double de=euclid(endX,endY,nodes[i].x,nodes[i].y);
        if (de<bestE){ bestE=de; eN=i; }
    }
    emit stageUpdate(
                QString("Start → node %1 at (%2, %3)")
                .arg(sN).arg(nodes[sN].x).arg(nodes[sN].y)
                );
    emit stageUpdate(
                QString("End   → node %1 at (%2, %3)")
                .arg(eN).arg(nodes[eN].x).arg(nodes[eN].y)
                );

    // BFS connectivity
    emit stageUpdate("Checking connectivity...");
    std::vector<bool> seen(nodes.size(),false);
    std::queue<int>   qq;
    seen[sN]=true; qq.push(sN);
    while(!qq.empty()) {
        int u=qq.front(); qq.pop();
        for (auto &ed: adj[u]) {
            if (!seen[ed.to]) {
                seen[ed.to]=true;
                qq.push(ed.to);
            }
        }
    }
    if (!seen[eN]) {
        emit stageUpdate("Graph NOT connected → abort");
        emit finished();
        emit routingComplete("Graph NOT connected → abort");
        return;
    }
    emit stageUpdate("Graph connected, proceeding...");

    // choose and run algorithm
    std::vector<int> prev(nodes.size(),-1), path;
    bool pathFound=false;
    QString suffix;

    if (algoType==Dijkstra) {
        emit stageUpdate("Running Dijkstra...");
        std::vector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
        using P = std::pair<double,int>;
        std::priority_queue<P,std::vector<P>,std::greater<P>> pq;
        dist[sN]=0; pq.push({0,sN});
        while(!pq.empty()) {
            auto [d,u]=pq.top(); pq.pop();
                    if (d>dist[u]) continue;
                    if (u==eN) break;
                    for (auto &ed: adj[u]) {
                double nd=d+ed.cost;
                if (nd<dist[ed.to]) {
                    dist[ed.to]=nd;
                    prev[ed.to]=u;
                    pq.push({nd,ed.to});
                }
            }
        }
        for (int at=eN; at!=-1; at=prev[at])
            path.push_back(at);
        if (!path.empty() && path.back()==sN) {
            std::reverse(path.begin(),path.end());
            emit stageUpdate(QString("Dijkstra: %1 nodes").arg(path.size()));
            pathFound=true; suffix="_dijkstra";
        } else {
            emit stageUpdate("Dijkstra: no path");
        }
    } else {
        emit stageUpdate("Running A*...");
        auto h = [&](int idx){
            return euclid(nodes[idx].x,nodes[idx].y, endX,endY);
        };
        struct Node{int idx; double f;};
        struct Cmp{ bool operator()(Node const&a,Node const&b) const {
                return a.f>b.f; }};
        std::vector<double> g(nodes.size(), std::numeric_limits<double>::infinity());
        std::priority_queue<Node,std::vector<Node>,Cmp> pq;
        g[sN]=0; pq.push({sN,h(sN)});
        while(!pq.empty()) {
            auto cur=pq.top(); pq.pop();
            int u=cur.idx;
            if (u==eN) break;
            if (cur.f - h(u) > g[u]) continue;
            for (auto &ed: adj[u]) {
                double tg=g[u]+ed.cost;
                if (tg<g[ed.to]) {
                    g[ed.to]=tg;
                    prev[ed.to]=u;
                    pq.push({ed.to, tg+h(ed.to)});
                }
            }
        }
        for (int at=eN; at!=-1; at=prev[at])
            path.push_back(at);
        if (!path.empty() && path.back()==sN) {
            std::reverse(path.begin(),path.end());
            emit stageUpdate(QString("A*: %1 nodes").arg(path.size()));
            pathFound=true; suffix="_astar";
        } else {
            emit stageUpdate("A*: no path");
            emit routingComplete("Error. No connectivity between start and end points.");
        }
    }

    if (pathFound) {
        // --- New: compute total path length ---
        // --- compute geodetic path length ---
        double totalMeters = 0.0;
        for (int i = 1; i < path.size(); ++i) {
            const PtKey &A = nodes[path[i-1]];
            const PtKey &B = nodes[path[i]];
            totalMeters += geodeticDistance(A.x, A.y, B.x, B.y);
        }
        double totalKm = totalMeters / 1000.0;
        emit stageUpdate(
                    QString("\nPath length: %1 m (%2 km)")
                    .arg(totalMeters, 0, 'f', 2)
                    .arg(totalKm,      0, 'f', 3)
                    );
        // --- End length computation ---

        // ensure output folder exists in current dir
        QDir curr = QDir::current();
        if (!curr.exists("RouteFinderOutput"))
            curr.mkdir("RouteFinderOutput");
        QString outDir = curr.filePath("RouteFinderOutput");
        QString timestamp = QDateTime::currentDateTime().toString("hhmmss");
        QString base = outDir + QDir::separator()
                + "route" + suffix + timestamp;
        QString shpPath = base + ".shp";
        QString csvPath = base + ".csv";

        // write shapefile with proper CRS and extent
            emit stageUpdate("Writing SHP to " + shpPath + "...");

            // Register drivers
            GDALAllRegister();

            // Get ESRI Shapefile driver
            GDALDriver *drv = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
            if (!drv) {
                emit stageUpdate("ERROR: ESRI Shapefile driver not available");
                emit finished();
                return;
            }

            // Remove existing file if it exists
            if (QFile::exists(shpPath)) {
                QFile::remove(shpPath);
            }

            // Create dataset
            GDALDataset *outDS = drv->Create(shpPath.toStdString().c_str(), 0, 0, 0, GDT_Unknown, nullptr);
            if (!outDS) {
                emit stageUpdate("ERROR: Cannot create shapefile");
                emit finished();
                return;
            }

            // Create spatial reference (WGS84)
            OGRSpatialReference srs;
            srs.SetWellKnownGeogCS("WGS84");

            // Create layer with LineString type and WGS84 CRS
            OGRLayer *lay = outDS->CreateLayer("route", &srs, wkbLineString, nullptr);
            if (!lay) {
                emit stageUpdate("ERROR: Cannot create layer");
                GDALClose(outDS);
                emit finished();
                return;
            }

            // Create line geometry
            OGRLineString line;
            for (int idx: path) {
                line.addPoint(nodes[idx].x, nodes[idx].y);
            }

            // Ensure the line has at least 2 points
            if (line.getNumPoints() < 2) {
                emit stageUpdate("ERROR: LineString must have at least 2 points");
                GDALClose(outDS);
                emit finished();
                return;
            }

            // Create feature and set geometry
            OGRFeature *of = OGRFeature::CreateFeature(lay->GetLayerDefn());
            of->SetGeometry(&line);

            // Create the feature in the layer
            if (lay->CreateFeature(of) != OGRERR_NONE) {
                emit stageUpdate("ERROR: Failed to create feature");
                OGRFeature::DestroyFeature(of);
                GDALClose(outDS);
                emit finished();
                return;
            }

            // Clean up
            OGRFeature::DestroyFeature(of);
            GDALClose(outDS);

            // Create the .prj file for WGS84
            QString prjPath = base + ".prj";
            QFile prjFile(prjPath);
            if (prjFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream prjStream(&prjFile);
                prjStream << "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AXIS[\"Latitude\",NORTH],AXIS[\"Longitude\",EAST],AUTHORITY[\"EPSG\",\"4326\"]]";
                prjFile.close();
            }

            emit stageUpdate("SHP written with WGS84 CRS.");

        // write CSV
        emit stageUpdate("Writing CSV to " + csvPath + "...");
        QFile csv(csvPath);
        if (csv.open(QIODevice::WriteOnly|QIODevice::Text)) {
            QTextStream ts(&csv);
            ts << "lon,lat\n";
            for (int idx: path) {
                ts << QString::number(nodes[idx].x,'f',6)
                   << "," << QString::number(nodes[idx].y,'f',6)
                   << "\n";
            }
            csv.close();
            emit stageUpdate("CSV written.");
        } else {
            emit stageUpdate("ERROR: cannot write CSV");
        }

        emit routingComplete(shpPath + ";" + csvPath);
    }

    emit finished();
}


#if 0 ///Old
void RoutingWorker::process()
{
    emit stageUpdate("Registering GDAL drivers...");
    GDALAllRegister();

    emit stageUpdate("Opening shapefile...");
    GDALDataset *inDS = (GDALDataset*)GDALOpenEx(
                inputPath.toStdString().c_str(),
                GDAL_OF_VECTOR, nullptr, nullptr, nullptr
                );
    if(!inDS) {
        emit stageUpdate("ERROR: cannot open shapefile.");
        emit finished(); return;
    }

    OGRLayer *layer = inDS->GetLayer(0);
    if(!layer || wkbFlatten(layer->GetGeomType())!=wkbLineString) {
        emit stageUpdate("ERROR: invalid geometry type.");
        GDALClose(inDS);
        emit finished(); return;
    }

    emit stageUpdate("Building graph...");
    std::unordered_map<PtKey,int> nodeMap;
    std::vector<PtKey> nodes;
    std::vector<std::vector<Edge>> adj;
    layer->ResetReading();
    OGRFeature *feat;
    while((feat = layer->GetNextFeature())!=nullptr) {
        auto *line = feat->GetGeometryRef()->toLineString();
        int nPt = line->getNumPoints();
        for(int i=0; i+1<nPt; ++i) {
            PtKey a{line->getX(i), line->getY(i)};
            PtKey b{line->getX(i+1), line->getY(i+1)};
            int ia, ib;
            auto itA = nodeMap.find(a);
            if(itA==nodeMap.end()) {
                ia = nodes.size();
                nodeMap[a] = ia;
                nodes.push_back(a);
                adj.emplace_back();
            } else ia = itA->second;
            auto itB = nodeMap.find(b);
            if(itB==nodeMap.end()) {
                ib = nodes.size();
                nodeMap[b] = ib;
                nodes.push_back(b);
                adj.emplace_back();
            } else ib = itB->second;

            double cost = euclid(a.x,a.y,b.x,b.y);
            adj[ia].push_back({ib,cost});
            adj[ib].push_back({ia,cost});
        }
        OGRFeature::DestroyFeature(feat);
    }
    GDALClose(inDS);
    emit stageUpdate(QString("Graph built: %1 nodes").arg(nodes.size()));
    if(nodes.empty()) {
        emit stageUpdate("ERROR: no nodes.");
        emit finished(); return;
    }

    // snap start/end
    emit stageUpdate("Snapping start/end to nearest nodes...");
    int sN=0, eN=0;
    double bestS=std::numeric_limits<double>::infinity();
    double bestE=std::numeric_limits<double>::infinity();
    for(int i=0; i<nodes.size(); ++i) {
        double ds = euclid(startX,startY, nodes[i].x, nodes[i].y);
        if(ds<bestS) { bestS=ds; sN=i; }
        double de = euclid(endX,endY, nodes[i].x, nodes[i].y);
        if(de<bestE) { bestE=de; eN=i; }
    }
    emit stageUpdate(QString("Start node %1, End node %2").arg(sN).arg(eN));

    // prepare for whichever algorithm
    std::vector<int>     prev(nodes.size(), -1);
    std::vector<int>     path;
    bool pathFound = false;

    if(algoType == Dijkstra) {
        emit stageUpdate("Running Dijkstra...");
        std::vector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
        using Pair = std::pair<double,int>;
        std::priority_queue<Pair,std::vector<Pair>,std::greater<Pair>> pq;
        dist[sN]=0; pq.push({0,sN});

        while(!pq.empty()) {
            auto [d,u] = pq.top(); pq.pop();
                    if(d>dist[u]) continue;
                    if(u==eN) break;
                    for(auto &ed: adj[u]) {
                double nd = d + ed.cost;
                if(nd < dist[ed.to]) {
                    dist[ed.to]   = nd;
                    prev[ed.to]   = u;
                    pq.push({nd, ed.to});
                }
            }
        }
        // reconstruct
        for(int at=eN; at!=-1; at=prev[at]) path.push_back(at);
        if(!path.empty() && path.back()==sN) {
            std::reverse(path.begin(), path.end());
            emit stageUpdate(QString("Dijkstra path length: %1").arg(path.size()));
            pathFound = true;
        } else {
            emit stageUpdate("Dijkstra: no path found.");
        }
    }
    else /* AStar */ {
        emit stageUpdate("Running A*...");
        auto heuristic = [&](int idx){
            return euclid(nodes[idx].x,nodes[idx].y, endX,endY);
        };
        struct Node { int idx; double f; };
        struct Cmp { bool operator()(Node const&a, Node const&b) const { return a.f>b.f; } };

        std::vector<double> g(nodes.size(), std::numeric_limits<double>::infinity());
        std::priority_queue<Node,std::vector<Node>,Cmp> pq;
        g[sN] = 0;
        pq.push({sN, heuristic(sN)});

        while(!pq.empty()) {
            Node cur = pq.top(); pq.pop();
            int u = cur.idx;
            if(u==eN) break;
            // skip if stale
            if(cur.f - heuristic(u) > g[u]) continue;
            for(auto &ed: adj[u]) {
                double tg = g[u] + ed.cost;
                if(tg < g[ed.to]) {
                    g[ed.to]     = tg;
                    prev[ed.to]  = u;
                    double f     = tg + heuristic(ed.to);
                    pq.push({ed.to, f});
                }
            }
        }
        // reconstruct
        for(int at=eN; at!=-1; at=prev[at]) path.push_back(at);
        if(!path.empty() && path.back()==sN) {
            std::reverse(path.begin(), path.end());
            emit stageUpdate(QString("A* path length: %1").arg(path.size()));
            pathFound = true;
        } else {
            emit stageUpdate("A*: no path found.");
        }
    }

    // write output if found
    QString suffix = (algoType==Dijkstra ? "_dijkstra" : "_astar");
    QString outPath = QFileInfo(inputPath).absolutePath()
            + "/route" + suffix + QDateTime::currentDateTime().toString("hhmmss") + ".shp";

    if(pathFound) {
        emit stageUpdate("Writing output" + suffix + "...");
        GDALDriver *drv = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        if(QFile::exists(outPath)) QFile::remove(outPath);
        GDALDataset *outDS = drv->Create(outPath.toStdString().c_str(),0,0,0,GDT_Unknown,nullptr);
        OGRLayer *outL = outDS->CreateLayer("route", nullptr, wkbLineString, nullptr);

        OGRLineString route;
        for(int idx: path) route.addPoint(nodes[idx].x, nodes[idx].y);

        OGRFeature *of = OGRFeature::CreateFeature(outL->GetLayerDefn());
        of->SetGeometry(&route);
        outL->CreateFeature(of);
        OGRFeature::DestroyFeature(of);
        GDALClose(outDS);

        emit stageUpdate("Written: " + outPath);
        emit routingComplete(outPath);
    }

    emit finished();
}
#endif
