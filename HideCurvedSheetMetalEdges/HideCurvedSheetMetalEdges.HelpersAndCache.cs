using TS = Tekla.Structures;
using TSD = Tekla.Structures.Drawing;
using TSP = Tekla.Structures.Plugins;

namespace HideCurvedSheetMetalEdges
{
    using System.Collections.Generic;

    using Tekla.Common.Geometry;
    using TS.DrawingPresentationModel;
    using TS.Geometry3d;

    public partial class HideCurvedSheetMetalEdges
    {
        #region Pomocnicze – dopasowanie / cache linii modelu

        private static bool ModelEdgeIsPresentInList(
            LinePrimitive edge,
            List<LinePrimitive> list)
        {
            foreach (var e in list)
            {
                bool sameEndPoints =
                    (edge.StartPoint.DistanceTo(e.StartPoint) < DrawingEpsilon &&
                     edge.EndPoint.DistanceTo(e.EndPoint)     < DrawingEpsilon)
                    || (edge.StartPoint.DistanceTo(e.EndPoint) < DrawingEpsilon &&
                        edge.EndPoint.DistanceTo(e.StartPoint) < DrawingEpsilon);

                if (sameEndPoints)
                    return true;
            }

            return false;
        }

        private static void SplitLinesInPrimitiveList(
            IList<PrimitiveBase> primitives,
            List<CachedLine> cachedLines,
            List<PrimitiveBase> output)
        {
            if (primitives == null || primitives.Count == 0)
                return;

            foreach (var primitive in primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive linePrimitive:
                    {
                        var splitLines = SplitLinePrimitiveByIntersections(linePrimitive, cachedLines);

                        foreach (var splitLine in splitLines)
                            output.Add(splitLine);

                        break;
                    }

                    case PrimitiveGroup group:
                    {
                        var newGroupPrimitives = new List<PrimitiveBase>(group.Primitives.Count);

                        SplitLinesInPrimitiveList(group.Primitives, cachedLines, newGroupPrimitives);

                        group.Primitives.Clear();
                        foreach (var p in newGroupPrimitives)
                            group.Primitives.Add(p);

                        output.Add(group);
                        break;
                    }

                    default:
                        output.Add(primitive);
                        break;
                }
            }
        }

        private static TSD.Part GetDrawingPart(int drawingId)
        {
            var identifier = new TS.Identifier(drawingId);
            var input = new TSP.DrawingPluginBase.InputDefinition(identifier, identifier);
            return TSD.Tools.InputDefinitionFactory.GetDrawingObject(input) as TSD.Part;
        }

        private static bool SegmentBoundingBoxesOverlap(LineSegment a, LineSegment b, double tolerance)
        {
            double aMinX = System.Math.Min(a.Point1.X, a.Point2.X) - tolerance;
            double aMaxX = System.Math.Max(a.Point1.X, a.Point2.X) + tolerance;
            double aMinY = System.Math.Min(a.Point1.Y, a.Point2.Y) - tolerance;
            double aMaxY = System.Math.Max(a.Point1.Y, a.Point2.Y) + tolerance;

            double bMinX = System.Math.Min(b.Point1.X, b.Point2.X) - tolerance;
            double bMaxX = System.Math.Max(b.Point1.X, b.Point2.X) + tolerance;
            double bMinY = System.Math.Min(b.Point1.Y, b.Point2.Y) - tolerance;
            double bMaxY = System.Math.Max(b.Point1.Y, b.Point2.Y) + tolerance;

            return aMinX <= bMaxX && aMaxX >= bMinX &&
                   aMinY <= bMaxY && aMaxY >= bMinY;
        }

        private static bool LinePrimitiveBoundingBoxesOverlap(
            LinePrimitive a,
            LinePrimitive b,
            double tolerance)
        {
            double aMinX = System.Math.Min(a.StartPoint.X, a.EndPoint.X) - tolerance;
            double aMaxX = System.Math.Max(a.StartPoint.X, a.EndPoint.X) + tolerance;
            double aMinY = System.Math.Min(a.StartPoint.Y, a.EndPoint.Y) - tolerance;
            double aMaxY = System.Math.Max(a.StartPoint.Y, a.EndPoint.Y) + tolerance;

            double bMinX = System.Math.Min(b.StartPoint.X, b.EndPoint.X) - tolerance;
            double bMaxX = System.Math.Max(b.StartPoint.X, b.EndPoint.X) + tolerance;
            double bMinY = System.Math.Min(b.StartPoint.Y, b.EndPoint.Y) - tolerance;
            double bMaxY = System.Math.Max(b.StartPoint.Y, b.EndPoint.Y) + tolerance;

            return aMinX <= bMaxX && aMaxX >= bMinX &&
                   aMinY <= bMaxY && aMaxY >= bMinY;
        }

        private static List<CachedLine> BuildCachedLines(Segment presentation)
        {
            var cachedLines = new List<CachedLine>();

            if (presentation?.Primitives == null)
                return cachedLines;

            foreach (var primitive in presentation.Primitives)
            {
                switch (primitive)
                {
                    case LinePrimitive lp:
                        cachedLines.Add(new CachedLine(lp));
                        break;

                    case PrimitiveGroup g:
                        CollectAllLinesFromGroup(g, cachedLines);
                        break;
                }
            }

            return cachedLines;
        }

        private static void CollectAllLinesFromGroup(PrimitiveGroup group, List<CachedLine> output)
        {
            if (group?.Primitives == null) return;

            foreach (var p in group.Primitives)
            {
                switch (p)
                {
                    case LinePrimitive lp:
                        output.Add(new CachedLine(lp));
                        break;

                    case PrimitiveGroup nested:
                        CollectAllLinesFromGroup(nested, output);
                        break;
                }
            }
        }

        private readonly struct CachedLine
        {
            public readonly LinePrimitive Primitive;
            public readonly LineSegment Segment;

            public CachedLine(LinePrimitive primitive)
            {
                Primitive = primitive;
                Segment = new LineSegment(
                    new Point(primitive.StartPoint.X, primitive.StartPoint.Y, 0),
                    new Point(primitive.EndPoint.X,   primitive.EndPoint.Y,   0)
                );
            }
        }

        private readonly struct ModelEdgePair
        {
            public readonly LinePrimitive ModelEdgeInDrawing;
            public readonly bool VisibleLine;

            public ModelEdgePair(LinePrimitive modelEdgeInDrawing, bool visibleLine)
            {
                ModelEdgeInDrawing = modelEdgeInDrawing;
                VisibleLine = visibleLine;
            }
        }

        private struct LineIntersections
        {
            public bool Line01;
            public bool Line05;
            public bool Line09;

            public LineIntersections(bool line01, bool line05, bool line09)
            {
                Line01 = line01;
                Line05 = line05;
                Line09 = line09;
            }
        }

        #endregion
    }
}
