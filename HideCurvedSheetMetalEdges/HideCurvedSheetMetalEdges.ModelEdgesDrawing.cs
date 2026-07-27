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

        private bool LinePrimitiveCanOverlapAnyEdgeToBeDeleted(
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

        private bool LinePrimitiveCanOverlapModelEdge(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge)
        {
            if (TryGetCollinearOverlap(
                    linePrimitive,
                    modelEdge,
                    out _,
                    out _,
                    out _))
            {
                return true;
            }

            // Wstępny filtr nie może być bardziej restrykcyjny niż właściwe
            // dopasowanie. Tekla potrafi zwrócić linię minimalnie przesuniętą
            // lub obróconą względem projekcji krawędzi modelowej.
            return LinePrimitiveApproximatelyOverlapsModelEdge(
                linePrimitive,
                modelEdge,
                true);
        }

        private bool LinePrimitiveOverlapsWithEdgeToBeDeleted(
            LinePrimitive linePrimitive,
            ModelEdgePair modelEdgeToBeDeleted)
        {
            if (TryGetCollinearOverlap(
                    linePrimitive,
                    modelEdgeToBeDeleted.ModelEdgeInDrawing,
                    out double overlapLength,
                    out double lineLength,
                    out double modelEdgeLength))
            {
                double requiredReferenceLength = this.IsCurvedSectionView
                    ? Math.Min(lineLength, modelEdgeLength)
                    : lineLength;

                if (overlapLength >= requiredReferenceLength * 0.8)
                    return true;
            }

            return LinePrimitiveApproximatelyOverlapsModelEdge(
                linePrimitive,
                modelEdgeToBeDeleted.ModelEdgeInDrawing,
                false);
        }

        private bool LinePrimitiveApproximatelyOverlapsModelEdge(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge,
            bool candidateOnly)
        {
            if (linePrimitive == null || modelEdge == null)
                return false;

            double tolerance = GetModelEdgeMatchingTolerance();

            if (!LinePrimitiveBoundingBoxesOverlap(
                    linePrimitive,
                    modelEdge,
                    tolerance))
            {
                return false;
            }

            var lineStart = new Point(
                linePrimitive.StartPoint.X,
                linePrimitive.StartPoint.Y,
                0.0);

            var lineEnd = new Point(
                linePrimitive.EndPoint.X,
                linePrimitive.EndPoint.Y,
                0.0);

            var edgeStart = new Point(
                modelEdge.StartPoint.X,
                modelEdge.StartPoint.Y,
                0.0);

            var edgeEnd = new Point(
                modelEdge.EndPoint.X,
                modelEdge.EndPoint.Y,
                0.0);

            var lineDirection = new Vector(
                lineEnd.X - lineStart.X,
                lineEnd.Y - lineStart.Y,
                0.0);

            var edgeDirection = new Vector(
                edgeEnd.X - edgeStart.X,
                edgeEnd.Y - edgeStart.Y,
                0.0);

            double lineLength = lineDirection.GetLength();
            double edgeLength = edgeDirection.GetLength();

            if (lineLength <= tolerance || edgeLength <= tolerance)
                return false;

            double directionCosine = Math.Abs(
                Vector.Dot(lineDirection, edgeDirection) /
                (lineLength * edgeLength));

            double maximumAngle = this.IsCurvedSectionView
                ? Math.PI / 12.0
                : Math.PI / 60.0;

            if (directionCosine < Math.Cos(maximumAngle))
                return false;

            var closestPoints = ClosestPointsBetweenSegments2D(
                lineStart,
                lineEnd,
                edgeStart,
                edgeEnd);

            if (closestPoints == null ||
                Distance.PointToPoint(
                    closestPoints.Value.A,
                    closestPoints.Value.B) > tolerance)
            {
                return false;
            }

            edgeDirection.Normalize();

            double lineStartParameter = Vector.Dot(
                new Vector(
                    lineStart.X - edgeStart.X,
                    lineStart.Y - edgeStart.Y,
                    0.0),
                edgeDirection);

            double lineEndParameter = Vector.Dot(
                new Vector(
                    lineEnd.X - edgeStart.X,
                    lineEnd.Y - edgeStart.Y,
                    0.0),
                edgeDirection);

            double overlapMinimum = Math.Max(
                Math.Min(lineStartParameter, lineEndParameter),
                0.0);

            double overlapMaximum = Math.Min(
                Math.Max(lineStartParameter, lineEndParameter),
                edgeLength);

            double overlapLength = overlapMaximum - overlapMinimum;
            if (overlapLength <= tolerance)
                return false;

            double requiredRatio;

            if (candidateOnly)
                requiredRatio = 0.15;
            else
                requiredRatio = this.IsCurvedSectionView ? 0.5 : 0.65;

            double requiredOverlap =
                Math.Min(lineLength, edgeLength) * requiredRatio;

            return overlapLength >= requiredOverlap;
        }

        private double GetModelEdgeMatchingTolerance()
        {
            if (this.IsCurvedSectionView)
                return Math.Max(this.ActiveDrawingEpsilon * 4.0, 0.25);

            // Dla zwykłego widoku tolerancja pozostaje mała, ale uwzględnia
            // zaokrąglenia współrzędnych prymitywów generowanych przez Teklę.
            return Math.Min(
                Math.Max(this.ActiveDrawingEpsilon * 3.0, 0.03),
                0.15);
        }

        private bool LinePrimitiveOverlapsWithModelEdge(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge)
        {
            if (!TryGetCollinearOverlap(
                    linePrimitive,
                    modelEdge,
                    out double overlapLength,
                    out double lineLength,
                    out double modelEdgeLength))
            {
                return false;
            }

            double requiredReferenceLength = this.IsCurvedSectionView
                ? Math.Min(lineLength, modelEdgeLength)
                : lineLength;

            return overlapLength >= requiredReferenceLength * 0.8;
        }

        private bool TryGetCollinearOverlap(
            LinePrimitive linePrimitive,
            LinePrimitive modelEdge,
            out double overlapLength,
            out double lineLength,
            out double modelEdgeLength)
        {
            overlapLength = 0.0;
            lineLength = 0.0;
            modelEdgeLength = 0.0;

            if (linePrimitive == null || modelEdge == null)
                return false;

            double tolerance = this.ActiveDrawingEpsilon;

            if (!LinePrimitiveBoundingBoxesOverlap(
                    linePrimitive,
                    modelEdge,
                    tolerance))
            {
                return false;
            }

            var edgeStart = new Point(
                modelEdge.StartPoint.X,
                modelEdge.StartPoint.Y);

            var edgeEnd = new Point(
                modelEdge.EndPoint.X,
                modelEdge.EndPoint.Y);

            var lineStart = new Point(
                linePrimitive.StartPoint.X,
                linePrimitive.StartPoint.Y);

            var lineEnd = new Point(
                linePrimitive.EndPoint.X,
                linePrimitive.EndPoint.Y);

            Line edgeLine = new(edgeStart, edgeEnd);

            if (Distance.PointToLine(lineStart, edgeLine) > tolerance ||
                Distance.PointToLine(lineEnd, edgeLine) > tolerance)
            {
                return false;
            }

            var edgeDirection = new Vector(
                edgeEnd.X - edgeStart.X,
                edgeEnd.Y - edgeStart.Y,
                0.0);

            modelEdgeLength = edgeDirection.GetLength();
            if (modelEdgeLength <= tolerance)
                return false;

            edgeDirection.Normalize();

            double lineStartParameter = Vector.Dot(
                new Vector(
                    lineStart.X - edgeStart.X,
                    lineStart.Y - edgeStart.Y,
                    0.0),
                edgeDirection);

            double lineEndParameter = Vector.Dot(
                new Vector(
                    lineEnd.X - edgeStart.X,
                    lineEnd.Y - edgeStart.Y,
                    0.0),
                edgeDirection);

            double lineMinimum = Math.Min(lineStartParameter, lineEndParameter);
            double lineMaximum = Math.Max(lineStartParameter, lineEndParameter);

            double overlapMinimum = Math.Max(lineMinimum, 0.0);
            double overlapMaximum = Math.Min(lineMaximum, modelEdgeLength);

            overlapLength = overlapMaximum - overlapMinimum;
            if (overlapLength <= tolerance)
                return false;

            lineLength = Distance.PointToPoint(lineStart, lineEnd);
            return lineLength > tolerance;
        }

        #endregion
    }
}
