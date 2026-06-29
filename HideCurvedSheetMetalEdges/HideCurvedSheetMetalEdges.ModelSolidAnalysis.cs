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
            Vector viewAxisZ)
        {
            var modelEdgesInDrawing = new List<ModelEdgePair>();

            TSM.Solid solid = selectedModelPart.GetSolid();
            FaceEnumerator faceEnum = solid.GetFaceEnumerator();

            while (faceEnum.MoveNext())
            {
                if (faceEnum.Current is not { } currentFace)
                    continue;

                var facesWithSimilarNormal = GetFacesWithSimilarNormal(currentFace, solid.GetFaceEnumerator());

                foreach (Face faceWithSimilarNormal in facesWithSimilarNormal)
                {
                    LineSegment commonEdge = GetCommonEdge(currentFace, faceWithSimilarNormal);
                    if (commonEdge == null)
                        continue;

                    double dot1 = Vector.Dot(currentFace.Normal,           viewAxisZ);
                    double dot2 = Vector.Dot(faceWithSimilarNormal.Normal, viewAxisZ);

                    if (dot1 * dot2 < 0)
                        continue;

                    Point transformedStartPoint = TransformationMatrix.Transform(commonEdge.StartPoint);
                    Point transformedEndPoint   = TransformationMatrix.Transform(commonEdge.EndPoint);

                    var commonEdgeInDrawing = new LinePrimitive(
                        new Vector2(transformedStartPoint.X / Scale, transformedStartPoint.Y / Scale),
                        new Vector2(transformedEndPoint.X   / Scale, transformedEndPoint.Y   / Scale));

                    Vector middleNormal = new(
                        (currentFace.Normal.X + faceWithSimilarNormal.Normal.X) / 2.0,
                        (currentFace.Normal.Y + faceWithSimilarNormal.Normal.Y) / 2.0,
                        (currentFace.Normal.Z + faceWithSimilarNormal.Normal.Z) / 2.0);

                    bool visibleLine = middleNormal.GetAngleBetween(viewAxisZ) < Degrees90 + SmallAngleAllowance;

                    if (!CommonEdgeIsPresentInModelEdges(commonEdgeInDrawing, visibleLine, modelEdgesInDrawing))
                        modelEdgesInDrawing.Add(new ModelEdgePair(commonEdgeInDrawing, visibleLine));
                }
            }

            return modelEdgesInDrawing;
        }

        private static List<Face> GetFacesWithSimilarNormal(Face currentFace, FaceEnumerator faceEnumerator)
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
            var commonVertexes = new List<Point>();

            var currentFaceVertexes   = GetFaceVertexes(currentFace);
            var similarNormalVertexes = GetFaceVertexes(faceWithSimilarNormal);

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
                case > 6:
                    return null;
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
            double maxT = Vector.Dot(new Vector(p1.X - p0.X, p1.Y - p0.Y, p1.Z - p0.Z), dir);

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

            return Distance.PointToPoint(start, end) < DrawingEpsilon ? null : new LineSegment(start, end);
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

        private static bool CommonEdgeIsPresentInModelEdges(
            LinePrimitive commonEdgeInDrawing,
            bool visibleLine,
            List<ModelEdgePair> modelEdgesInDrawing)
        {
            foreach (ModelEdgePair modelEdge in modelEdgesInDrawing)
            {
                bool sameEndPoints =
                    (commonEdgeInDrawing.StartPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.StartPoint) < DrawingEpsilon &&
                     commonEdgeInDrawing.EndPoint.DistanceTo(modelEdge.ModelEdgeInDrawing.EndPoint)   < DrawingEpsilon)
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
