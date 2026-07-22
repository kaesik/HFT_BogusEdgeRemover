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
            SolidAnalysisCacheEntry solidCache = GetOrBuildSolidAnalysisCache(selectedModelPart);

            foreach (LineSegment modelEdge in solidCache.AllModelEdges)
            {
                List<LinePrimitive> projectedEdges =
                    ProjectModelEdgeToPresentation(modelEdge);

                foreach (LinePrimitive edgeInDrawing in projectedEdges)
                {
                    if (!ModelEdgeIsPresentInList(edgeInDrawing, modelEdgesInDrawing))
                        modelEdgesInDrawing.Add(edgeInDrawing);
                }
            }

            return modelEdgesInDrawing;
        }

        private static bool LinePrimitiveCanOverlapAnyEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            List<ModelEdgePair> modelEdgesToBeDeleted)
        {
            if (linePrimitive == null || modelEdgesToBeDeleted == null || modelEdgesToBeDeleted.Count == 0)
                return false;

            foreach (var modelEdge in modelEdgesToBeDeleted)
            {
                if (LinePrimitiveCanOverlapModelEdge(linePrimitive, modelEdge.ModelEdgeInDrawing))
                    return true;
            }

            return false;
        }

        private static bool LinePrimitiveCanOverlapModelEdge(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge)
        {
            if (linePrimitive == null || modelEdge == null)
                return false;

            if (!LinePrimitiveBoundingBoxesOverlap(linePrimitive, modelEdge, DrawingEpsilon))
                return false;

            var edgeStart = new Point(modelEdge.StartPoint.X, modelEdge.StartPoint.Y);
            var edgeEnd   = new Point(modelEdge.EndPoint.X,   modelEdge.EndPoint.Y);

            Line edgeLine = new(edgeStart, edgeEnd);

            var lpStart = new Point(linePrimitive.StartPoint.X, linePrimitive.StartPoint.Y);
            var lpEnd   = new Point(linePrimitive.EndPoint.X,   linePrimitive.EndPoint.Y);

            if (Distance.PointToLine(lpStart, edgeLine) > DrawingEpsilon ||
                Distance.PointToLine(lpEnd,   edgeLine) > DrawingEpsilon)
            {
                return false;
            }

            var edgeDir = new Vector(edgeEnd.X - edgeStart.X, edgeEnd.Y - edgeStart.Y, 0);
            double edgeLen = edgeDir.GetLength();
            if (edgeLen < DrawingEpsilon)
                return false;

            edgeDir.Normalize();

            double tLp0 = Vector.Dot(new Vector(lpStart.X - edgeStart.X, lpStart.Y - edgeStart.Y, 0), edgeDir);
            double tLp1 = Vector.Dot(new Vector(lpEnd.X   - edgeStart.X, lpEnd.Y   - edgeStart.Y, 0), edgeDir);

            double lpMin = Math.Min(tLp0, tLp1);
            double lpMax = Math.Max(tLp0, tLp1);

            double overlapMin = Math.Max(lpMin, 0.0);
            double overlapMax = Math.Min(lpMax, edgeLen);

            return overlapMax - overlapMin > DrawingEpsilon;
        }

        private static bool LinePrimitiveOverlapsWithEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            ModelEdgePair modelEdgeToBeDeleted)
        {
            var edge = modelEdgeToBeDeleted.ModelEdgeInDrawing;

            if (!LinePrimitiveBoundingBoxesOverlap(linePrimitive, edge, DrawingEpsilon))
                return false;

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
            if (!LinePrimitiveBoundingBoxesOverlap(linePrimitive, modelEdge, DrawingEpsilon))
                return false;

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
