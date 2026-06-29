using TS = Tekla.Structures;
using TSM = Tekla.Structures.Model;

namespace HideCurvedSheetMetalEdges
{
    using System;
    using System.Collections.Generic;

    using Tekla.Common.Geometry;
    using TS.DrawingPresentationModel;
    using TS.Geometry3d;
    using TS.Solid;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Krawędzie modelu – zbieranie / dopasowanie do rysunku

        private List<LinePrimitive> GetModelEdgesInDrawingToKeep(
            TSM.Part selectedModelPart)
        {
            var modelEdgesInDrawing = new List<LinePrimitive>();

            TSM.Solid solid = selectedModelPart.GetSolid();
            FaceEnumerator faceEnum = solid.GetFaceEnumerator();

            while (faceEnum.MoveNext())
            {
                if (faceEnum.Current is not { } currentFace)
                    continue;

                LoopEnumerator loopEnum = currentFace.GetLoopEnumerator();

                while (loopEnum.MoveNext())
                {
                    if (loopEnum.Current is not { } loop)
                        continue;

                    var vertices = new List<Point>();
                    VertexEnumerator vertexEnum = loop.GetVertexEnumerator();

                    while (vertexEnum.MoveNext())
                    {
                        Point vertex = vertexEnum.Current;
                        if (vertex != null)
                            vertices.Add(vertex);
                    }

                    int count = vertices.Count;
                    if (count < 2)
                        continue;

                    for (int i = 0; i < count; i++)
                    {
                        Point p0 = vertices[i];
                        Point p1 = vertices[(i + 1) % count];

                        if (Distance.PointToPoint(p0, p1) < ModelEpsilon * 0.5)
                            continue;

                        Point t0 = TransformationMatrix.Transform(p0);
                        Point t1 = TransformationMatrix.Transform(p1);

                        var edgeInDrawing = new LinePrimitive(
                            new Vector2(t0.X / Scale, t0.Y / Scale),
                            new Vector2(t1.X / Scale, t1.Y / Scale));

                        if (!ModelEdgeIsPresentInList(edgeInDrawing, modelEdgesInDrawing))
                            modelEdgesInDrawing.Add(edgeInDrawing);
                    }
                }
            }

            return modelEdgesInDrawing;
        }

        private static bool LinePrimitiveOverlapsWithEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            ModelEdgePair modelEdgeToBeDeleted)
        {
            var edge = modelEdgeToBeDeleted.ModelEdgeInDrawing;

            Line edgeLine = new(
                new Point(edge.StartPoint.X, edge.StartPoint.Y),
                new Point(edge.EndPoint.X,   edge.EndPoint.Y));

            var lpStart = new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y);
            var lpEnd   = new Point(linePrimitive.EndPoint.X,   linePrimitive.EndPoint.Y);

            if (Distance.PointToLine(lpStart, edgeLine) > DrawingEpsilon ||
                Distance.PointToLine(lpEnd,   edgeLine) > DrawingEpsilon)
                return false;

            var e0 = new Point(edge.StartPoint.X, edge.StartPoint.Y);
            var e1 = new Point(edge.EndPoint.X,   edge.EndPoint.Y);

            var edgeDir = new Vector(e1.X - e0.X, e1.Y - e0.Y, 0);
            double edgeLen = edgeDir.GetLength();
            if (edgeLen < DrawingEpsilon)
                return false;

            var edgeDirUnit = new Vector(edgeDir);
            edgeDirUnit.Normalize();

            double tLp0 = Vector.Dot(new Vector(lpStart.X - e0.X, lpStart.Y - e0.Y, 0), edgeDirUnit);
            double tLp1 = Vector.Dot(new Vector(lpEnd.X   - e0.X, lpEnd.Y   - e0.Y, 0), edgeDirUnit);

            double lpMin = Math.Min(tLp0, tLp1);
            double lpMax = Math.Max(tLp0, tLp1);

            double overlapMin = Math.Max(lpMin, 0.0);
            double overlapMax = Math.Min(lpMax, edgeLen);

            if (overlapMax - overlapMin <= DrawingEpsilon)
                return false;

            double overlapLen = overlapMax - overlapMin;
            double lpLen = Distance.PointToPoint(lpStart, lpEnd);

            return overlapLen >= lpLen * 0.8;
        }

        private static bool LinePrimitiveOverlapsWithModelEdge(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge)
        {
            Line edgeLine = new(
                new Point(modelEdge.StartPoint.X, modelEdge.StartPoint.Y),
                new Point(modelEdge.EndPoint.X,   modelEdge.EndPoint.Y));

            var lpStart = new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y);
            var lpEnd   = new Point(linePrimitive.EndPoint.X,   linePrimitive.EndPoint.Y);

            if (Distance.PointToLine(lpStart, edgeLine) > DrawingEpsilon ||
                Distance.PointToLine(lpEnd,   edgeLine) > DrawingEpsilon)
                return false;

            var e0 = new Point(modelEdge.StartPoint.X, modelEdge.StartPoint.Y);
            var e1 = new Point(modelEdge.EndPoint.X,   modelEdge.EndPoint.Y);

            var edgeDir = new Vector(e1.X - e0.X, e1.Y - e0.Y, 0);
            double edgeLen = edgeDir.GetLength();
            if (edgeLen < DrawingEpsilon)
                return false;

            var edgeDirUnit = new Vector(edgeDir);
            edgeDirUnit.Normalize();

            double tLp0 = Vector.Dot(new Vector(lpStart.X - e0.X, lpStart.Y - e0.Y, 0), edgeDirUnit);
            double tLp1 = Vector.Dot(new Vector(lpEnd.X   - e0.X, lpEnd.Y   - e0.Y, 0), edgeDirUnit);

            double lpMin = Math.Min(tLp0, tLp1);
            double lpMax = Math.Max(tLp0, tLp1);

            double overlapMin = Math.Max(lpMin, 0.0);
            double overlapMax = Math.Min(lpMax, edgeLen);

            if (overlapMax - overlapMin <= DrawingEpsilon)
                return false;

            double overlapLen = overlapMax - overlapMin;
            double lpLen = Distance.PointToPoint(lpStart, lpEnd);

            return overlapLen >= lpLen * 0.8;
        }

        #endregion
    }
}
