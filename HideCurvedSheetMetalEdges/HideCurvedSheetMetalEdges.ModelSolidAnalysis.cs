using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;

namespace HideCurvedSheetMetalEdges
{
    using System.Collections.Generic;
    using System.Linq;

    using Tekla.Common.Geometry;
    using TS.DrawingPresentationModel;
    using TS.Geometry3d;
    using TS.Solid;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Analiza bryły modelu – krawędzie do usunięcia

        private List<ModelEdgePair> GetModelEdgesInDrawingToBeDeletedInDrawing(
            TSM.Part selectedModelPart,
            Vector viewAxisZ,
            bool curvedSectionView)
        {
            var modelEdgesInDrawing = new List<ModelEdgePair>();
            SolidAnalysisCacheEntry solidCache = GetOrBuildSolidAnalysisCache(selectedModelPart);

            foreach (var candidate in solidCache.CandidateEdgesToDelete)
            {
                double firstDotViewAxisZ = Vector.Dot(candidate.FirstFaceNormal, viewAxisZ);
                double secondDotViewAxisZ = Vector.Dot(candidate.SecondFaceNormal, viewAxisZ);

                if (!curvedSectionView &&
                    firstDotViewAxisZ * secondDotViewAxisZ < 0)
                {
                    continue;
                }

                bool visibleLine =
                    candidate.MiddleNormal.GetAngleBetween(viewAxisZ) <
                    Degrees90 + SmallAngleAllowance;

                List<LinePrimitive> projectedEdges =
                    ProjectModelEdgeToPresentation(candidate.Edge);

                foreach (LinePrimitive commonEdgeInDrawing in projectedEdges)
                {
                    if (CommonEdgeIsPresentInModelEdges(
                            commonEdgeInDrawing,
                            visibleLine,
                            modelEdgesInDrawing))
                    {
                        continue;
                    }

                    modelEdgesInDrawing.Add(
                        new ModelEdgePair(commonEdgeInDrawing, visibleLine));
                }
            }

            return modelEdgesInDrawing;
        }

        private static SolidAnalysisCacheEntry BuildSolidAnalysisCacheEntry(TSM.Part selectedModelPart)
        {
            TSM.Solid solid = selectedModelPart.GetSolid();

            List<SolidFaceCache> faces =
                GetSolidFacesWithCachedVertexes(solid, out var allModelEdges);

            var candidateEdgesToDelete = new List<ModelEdgeCandidate>();

            for (int i = 0; i < faces.Count; i++)
            {
                SolidFaceCache currentFace = faces[i];

                for (int j = i + 1; j < faces.Count; j++)
                {
                    SolidFaceCache faceWithSimilarNormal = faces[j];

                    double normalAngle =
                        currentFace.Normal.GetAngleBetween(faceWithSimilarNormal.Normal);

                    if (normalAngle > BigAngleAllowance)
                        continue;

                    LineSegment commonEdge = GetCommonEdge(
                        currentFace.Vertexes,
                        faceWithSimilarNormal.Vertexes);

                    if (commonEdge == null)
                        continue;

                    var middleNormal = new Vector(
                        (currentFace.Normal.X + faceWithSimilarNormal.Normal.X) / 2.0,
                        (currentFace.Normal.Y + faceWithSimilarNormal.Normal.Y) / 2.0,
                        (currentFace.Normal.Z + faceWithSimilarNormal.Normal.Z) / 2.0);

                    candidateEdgesToDelete.Add(new ModelEdgeCandidate(
                        commonEdge,
                        currentFace.Normal,
                        faceWithSimilarNormal.Normal,
                        middleNormal));
                }
            }

            return new SolidAnalysisCacheEntry(
                candidateEdgesToDelete,
                allModelEdges);
        }

        private static List<SolidFaceCache> GetSolidFacesWithCachedVertexes(
            TSM.Solid solid,
            out List<LineSegment> allModelEdges)
        {
            var faces = new List<SolidFaceCache>();
            allModelEdges = new List<LineSegment>();

            FaceEnumerator faceEnum = solid.GetFaceEnumerator();

            while (faceEnum.MoveNext())
            {
                if (faceEnum.Current is not { } face)
                    continue;

                var faceVertexes = new List<Point>();
                LoopEnumerator loopEnum = face.GetLoopEnumerator();

                while (loopEnum.MoveNext())
                {
                    if (loopEnum.Current is not { } loop)
                        continue;

                    var loopVertexes = new List<Point>();
                    VertexEnumerator vertexEnum = loop.GetVertexEnumerator();

                    while (vertexEnum.MoveNext())
                    {
                        Point vertex = vertexEnum.Current;
                        if (vertex == null)
                            continue;

                        loopVertexes.Add(vertex);
                        faceVertexes.Add(vertex);
                    }

                    int loopVertexCount = loopVertexes.Count;
                    if (loopVertexCount < 2)
                        continue;

                    for (int i = 0; i < loopVertexCount; i++)
                    {
                        Point p0 = loopVertexes[i];
                        Point p1 = loopVertexes[(i + 1) % loopVertexCount];

                        if (Distance.PointToPoint(p0, p1) < ModelEpsilon * 0.5)
                            continue;

                        allModelEdges.Add(new LineSegment(p0, p1));
                    }
                }

                if (faceVertexes.Count < 2)
                    continue;

                faces.Add(new SolidFaceCache(
                    new Vector(face.Normal),
                    faceVertexes));
            }

            return faces;
        }

        private static List<Face> GetFacesWithSimilarNormal(
            Face currentFace,
            FaceEnumerator faceEnumerator)
        {
            var facesWithSimilarNormal = new List<Face>();

            while (faceEnumerator.MoveNext())
            {
                if (faceEnumerator.Current is not { } secondaryFace)
                    continue;

                if (ReferenceEquals(secondaryFace, currentFace) || secondaryFace.Equals(currentFace))
                    continue;

                double normalAngle = currentFace.Normal.GetAngleBetween(secondaryFace.Normal);
                if (normalAngle <= BigAngleAllowance)
                    facesWithSimilarNormal.Add(secondaryFace);
            }

            return facesWithSimilarNormal;
        }

        private static LineSegment GetCommonEdge(Face currentFace, Face faceWithSimilarNormal)
        {
            return GetCommonEdge(
                GetFaceVertexes(currentFace),
                GetFaceVertexes(faceWithSimilarNormal));
        }

        private static LineSegment GetCommonEdge(
            List<Point> currentFaceVertexes,
            List<Point> similarNormalVertexes)
        {
            var commonVertexes = new List<Point>();

            foreach (Point currentVertex in currentFaceVertexes)
            {
                foreach (Point similarNormalVertex in similarNormalVertexes)
                {
                    if (Distance.PointToPoint(currentVertex, similarNormalVertex) < ModelEpsilon)
                    {
                        bool exists = commonVertexes.Any(v =>
                            Distance.PointToPoint(v, currentVertex) < ModelEpsilon * 0.5);

                        if (!exists)
                            commonVertexes.Add(currentVertex);
                    }
                }
            }

            int count = commonVertexes.Count;

            switch (count)
            {
                case < 2:
                    return null;
                case 2:
                    return new LineSegment(commonVertexes[0], commonVertexes[1]);
            }

            Point p0 = commonVertexes[0];

            Point p1 = null;
            double maxDist = 0.0;

            for (int i = 1; i < count; i++)
            {
                double d = Distance.PointToPoint(p0, commonVertexes[i]);
                if (d > ModelEpsilon * 0.5 && d > maxDist)
                {
                    maxDist = d;
                    p1 = commonVertexes[i];
                }
            }

            if (p1 == null)
                return null;

            var dir = new Vector(p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z);
            double dirLen = dir.GetLength();
            if (dirLen < ModelEpsilon * 0.5)
                return null;

            dir.Normalize();

            double minT = 0.0;
            double maxT = Vector.Dot(
                new Vector(p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z),
                dir);

            foreach (var p in commonVertexes)
            {
                var v = new Vector(p.X - p0.X, p.Y - p0.Y, p.Z - p0.Z);
                double t = Vector.Dot(v, dir);

                var closest = new Point(
                    p0.X + dir.X * t,
                    p0.Y + dir.Y * t,
                    p0.Z + dir.Z * t);

                double off = Distance.PointToPoint(p, closest);

                if (off > ModelEpsilon * 0.5)
                    return null;

                if (t < minT) minT = t;
                if (t > maxT) maxT = t;
            }

            var start = new Point(
                p0.X + dir.X * minT,
                p0.Y + dir.Y * minT,
                p0.Z + dir.Z * minT);

            var end = new Point(
                p0.X + dir.X * maxT,
                p0.Y + dir.Y * maxT,
                p0.Z + dir.Z * maxT);

            return Distance.PointToPoint(start, end) < ModelEpsilon * 0.5
                ? null
                : new LineSegment(start, end);
        }

        private static List<Point> GetFaceVertexes(Face currentFace)
        {
            var faceVertexes = new List<Point>();
            LoopEnumerator loopEnum = currentFace.GetLoopEnumerator();

            while (loopEnum.MoveNext())
            {
                if (loopEnum.Current is not { } loop)
                    continue;

                VertexEnumerator vertexEnum = loop.GetVertexEnumerator();
                while (vertexEnum.MoveNext())
                {
                    Point vertex = vertexEnum.Current;
                    if (vertex != null)
                        faceVertexes.Add(vertex);
                }
            }

            return faceVertexes;
        }

        private readonly struct SolidFaceCache
        {
            public readonly Vector Normal;
            public readonly List<Point> Vertexes;

            public SolidFaceCache(Vector normal, List<Point> vertexes)
            {
                Normal = normal;
                Vertexes = vertexes;
            }
        }

        private static bool CommonEdgeIsPresentInModelEdges(
            LinePrimitive commonEdgeInDrawing,
            bool visibleLine,
            List<ModelEdgePair> modelEdgesInDrawing)
        {
            foreach (ModelEdgePair modelEdge in modelEdgesInDrawing)
            {
                bool sameEndPoints =
                    (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon &&
                     commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint)     < DrawingEpsilon)
                    || (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint) < DrawingEpsilon &&
                        commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon);

                if (sameEndPoints && modelEdge.VisibleLine == visibleLine)
                    return true;
            }

            return false;
        }

        #endregion
    }
}
